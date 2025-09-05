#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

bridge = CvBridge()
pub = None

def image_callback(msg):
    global pub
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Detect yellow parcel (example)
    lower = np.array([15, 60, 60])
    upper = np.array([35, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        cnt = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(cnt)
        cx, cy = x + w/2, y + h/2

        # For now, treat pixel coords as x,y
        point = Point()
        point.x = cx
        point.y = cy
        point.z = 0.0
        pub.publish(point)

    cv2.imshow("Frame", frame)
    cv2.waitKey(1)

def main():
    global pub
    rospy.init_node("vision_node")
    rospy.Subscriber("/camera/image_raw", Image, image_callback)
    pub = rospy.Publisher("/parcel/position", Point, queue_size=10)
    rospy.spin()

if __name__ == "__main__":
    main()
