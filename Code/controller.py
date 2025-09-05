#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import Point, Pose

class ArmController:
    def __init__(self):
        rospy.init_node("arm_controller", anonymous=True)

        # Initialize MoveIt commander
        moveit_commander.roscpp_initialize([])
        self.arm = moveit_commander.MoveGroupCommander("arm")   # matches your MoveIt group name
        self.gripper = moveit_commander.MoveGroupCommander("gripper")

        rospy.Subscriber("/parcel_position", Point, self.pick_and_place)

    def pick_and_place(self, msg):
        rospy.loginfo(f"Received parcel position: x={msg.x}, y={msg.y}, z={msg.z}")

        # Define target pose
        target_pose = Pose()
        target_pose.position.x = msg.x / 1000.0  # convert mm/px → meters
        target_pose.position.y = msg.y / 1000.0
        target_pose.position.z = 0.1  # safe height above parcel
        target_pose.orientation.w = 1.0

        # Move above the parcel
        self.arm.set_pose_target(target_pose)
        self.arm.go(wait=True)

        # Lower down to grab
        target_pose.position.z = 0.0
        self.arm.set_pose_target(target_pose)
        self.arm.go(wait=True)

        # Close gripper
        self.gripper.set_named_target("close")
        self.gripper.go(wait=True)

        # Lift parcel
        target_pose.position.z = 0.2
        self.arm.set_pose_target(target_pose)
        self.arm.go(wait=True)

        # Place in bin (example location)
        place_pose = Pose()
        place_pose.position.x = 0.2
        place_pose.position.y = -0.2
        place_pose.position.z = 0.1
        place_pose.orientation.w = 1.0

        self.arm.set_pose_target(place_pose)
        self.arm.go(wait=True)

        # Open gripper
        self.gripper.set_named_target("open")
        self.gripper.go(wait=True)

if __name__ == "__main__":
    try:
        ArmController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
