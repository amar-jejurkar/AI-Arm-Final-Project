#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

class ParcelDetector:
    def __init__(self):
        rospy.init_node("parcel_detector", anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.coord_pub = rospy.Publisher("/parcel_position", Point, queue_size=10)

    def callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Example: detect red parcels
        lower_red = (0, 100, 100)
        upper_red = (10, 255, 255)
        mask = cv2.inRange(hsv, lower_red, upper_red)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 5000:   # ignore small noise
                x, y, w, h = cv2.boundingRect(cnt)
                cx, cy = x + w // 2, y + h // 2
                cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)

                # Publish center position as ROS Point
                point = Point()
                point.x, point.y, point.z = cx, cy, 0.0
                self.coord_pub.publish(point)

        cv2.imshow("Parcel Detection", frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    try:
        ParcelDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass




#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ArmController:
    def __init__(self):
        rospy.init_node("arm_controller", anonymous=True)
        rospy.Subscriber("/parcel_position", Point, self.callback)
        self.arm_pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=10)

    def callback(self, data):
        # Simple mapping (cx,cy) → arm movement
        traj = JointTrajectory()
        traj.joint_names = ["joint1", "joint2", "joint3", "joint4", "gripper"]

        point = JointTrajectoryPoint()
        # Example dummy values (replace with IK calculations)
        point.positions = [0.5, 0.3, 0.2, -0.1, 0.0]  
        point.time_from_start = rospy.Duration(2)

        traj.points.append(point)
        self.arm_pub.publish(traj)

if __name__ == "__main__":
    try:
        ArmController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


