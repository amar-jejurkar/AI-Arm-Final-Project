


import cv2
import numpy as np
import serial
import time
import argparse
import math

# -------------------------
# USER CONFIGURABLE VALUES
# -------------------------
SERIAL_PORT = "/dev/ttyUSB0"      # change to your Arduino serial port
BAUDRATE = 115200

# Robot arm link lengths in meters (example values)
L1 = 0.15   # shoulder -> elbow
L2 = 0.15   # elbow -> wrist (end effector)
# height offset of camera relative to robot base (meters)
CAM_HEIGHT = 0.30

# Servo angle limits (degrees) and mapping to servo PWM if needed
SERVO_MIN = 0    # deg
SERVO_MAX = 180  # deg

# Calibration: pixel -> world mapping
# We'll compute an affine transform from 4 calibration points (px,py) -> (mx,my) in meters.
# Put your measured correspondences in the lists below.
CALIB_PIX = np.array([
    [100, 50],
    [540, 50],
    [540, 380],
    [100, 380]
], dtype=np.float32)

CALIB_WORLD = np.array([
    [0.05, 0.10],
    [0.35, 0.10],
    [0.35, 0.30],
    [0.05, 0.30]
], dtype=np.float32)

# Detection thresholds (example: detect yellow parcels)
LOWER_HSV = np.array([15, 60, 60])   # adjust according to parcel color or use other model
UPPER_HSV = np.array([35, 255, 255])

# Minimum contour area to be considered a parcel
MIN_AREA = 5000

# -------------------------
# UTILITIES
# -------------------------
def compute_affine_transform(src_pts_px, dst_pts_m):
    """Compute a 3x3 homography/affine that maps pixel coords to meter coords.
       We'll use a perspective transform (cv2.getPerspectiveTransform expects 4 points).
    """
    M = cv2.getPerspectiveTransform(src_pts_px, dst_pts_m)
    return M

def pixel_to_world(px, py, M):
    """Transform pixel coordinates (px,py) to world coordinates (mx,my) using homography M."""
    src = np.array([[[px, py]]], dtype=np.float32)
    dst = cv2.perspectiveTransform(src, M)
    mx, my = dst[0,0]
    return float(mx), float(my)

# -------------------------
# INVERSE KINEMATICS (2-link planar arm)
# -------------------------
def inverse_kinematics_2link(x, y, l1=L1, l2=L2):
    """
    Solve IK for a planar 2-link arm (shoulder, elbow) with base at (0,0).
    Returns angles in degrees: theta1 (shoulder), theta2 (elbow).
    Derivation:
      cos_theta2 = (x^2 + y^2 - l1^2 - l2^2) / (2*l1*l2)
      theta2 = atan2( +/- sqrt(1 - cos^2), cos_theta2 )
      theta1 = atan2(y, x) - atan2(l2*sin(theta2), l1 + l2*cos(theta2))
    Choose elbow-down solution (negative sign on sqrt).
    """
    # clamp reachable
    r2 = x*x + y*y
    denom = 2 * l1 * l2
    cos_theta2 = (r2 - l1*l1 - l2*l2) / denom
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))
    # Choose elbow-down (negative sqrt). If you want elbow-up use +.
    sin_theta2 = -math.sqrt(max(0.0, 1 - cos_theta2*cos_theta2))
    theta2 = math.atan2(sin_theta2, cos_theta2)

    k1 = l1 + l2 * math.cos(theta2)
    k2 = l2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)

    # convert to degrees
    deg1 = math.degrees(theta1)
    deg2 = math.degrees(theta2)

    # Normalize angles to 0-180 (example mapping; depends on servo mounting)
    # You may need to offset and clamp according to your hardware orientation
    return deg1, deg2

def map_angle_to_servo(angle_deg, offset=0.0):
    """Map angle (degrees) to a servo friendly range (0-180). Add offset if servo mounting requires it."""
    a = angle_deg + offset
    a = max(SERVO_MIN, min(SERVO_MAX, a))
    return int(round(a))

# -------------------------
# SERIAL SENDER
# -------------------------
class ArmSerial:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # wait for Arduino reset

    def send_angles(self, base_angle, shoulder_angle, elbow_angle):
        """
        Protocol: send as text line "B:base,S:shoulder,E:elbow\n"
        angles integers 0..180
        """
        line = f"B:{int(base_angle)},S:{int(shoulder_angle)},E:{int(elbow_angle)}\n"
        self.ser.write(line.encode('utf-8'))
        # optionally read ack
        # ack = self.ser.readline().decode().strip()
        # print("ACK:", ack)

# -------------------------
# MAIN: capture, detect, compute, send
# -------------------------
def main(args):
    # parse args for port override
    port = args.port or SERIAL_PORT
    baud = args.baud or BAUDRATE

    # Open serial
    arm = ArmSerial(port, baud)
    print(f"[INFO] Serial opened on {port} @ {baud}")

    # Compute transform
    M = compute_affine_transform(CALIB_PIX, CALIB_WORLD)

    # Open camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera")

    print("[INFO] Starting capture loop. Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # resize for speed
        frame_small = cv2.resize(frame, (640, 480))
        hsv = cv2.cvtColor(frame_small, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        target_center_px = None

        if contours:
            # choose largest contour above area threshold
            contours = sorted(contours, key=cv2.contourArea, reverse=True)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < MIN_AREA:
                    continue
                x,y,w,h = cv2.boundingRect(cnt)
                cx = x + w/2
                cy = y + h/2
                target_center_px = (cx, cy)
                # draw
                cv2.rectangle(frame_small, (x,y), (x+w, y+h), (0,255,0), 2)
                cv2.circle(frame_small, (int(cx), int(cy)), 5, (0,0,255), -1)
                break

        # If detected compute world coords and IK
        if target_center_px is not None:
            px, py = target_center_px
            # transform pixel to world (meters)
            mx, my = pixel_to_world(px, py, M)   # my is along camera's y axis -> robot y
            # We assume robot base XY plane: use mx,my as x,y in robot plane.
            # If camera is angled, then include height to compute projection properly (more complex)
            print(f"Detected parcel at pixel ({px:.1f},{py:.1f}) -> world ({mx:.3f}m, {my:.3f}m)")

            # For 2-link planar IK we need coordinates relative to base.
            # If world coords are given relative to base, OK. If not, apply offsets here.
            target_x = mx
            target_y = my

            # check reachable
            dist = math.hypot(target_x, target_y)
            if dist > (L1 + L2) or dist < abs(L1 - L2):
                print("[WARN] target out of reach:", dist)
            else:
                shoulder_deg, elbow_deg = inverse_kinematics_2link(target_x, target_y)
                # base rotation: for this simple planar arm assume base is aligned and base rotation is 0.
                # If your arm has a rotating base (servo0), compute base angle from camera-to-arm yaw:
                base_deg = 90  # example neutral position; adapt if needed
                # map angles to servo ranges and add offsets if required
                servo_base = map_angle_to_servo(base_deg, offset=0)
                servo_shoulder = map_angle_to_servo(shoulder_deg, offset=90)  # offset depends on mount
                servo_elbow = map_angle_to_servo(elbow_deg, offset=90)

                print(f"Angles -> base:{servo_base} shoulder:{servo_shoulder} elbow:{servo_elbow}")

                # Send to Arduino
                arm.send_angles(servo_base, servo_shoulder, servo_elbow)

        # show windows
        cv2.imshow("frame", frame_small)
        cv2.imshow("mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    arm.ser.close()

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port", default=None)
    ap.add_argument("--baud", type=int, help="baud rate", default=None)
    args = ap.parse_args()
    main(args)
