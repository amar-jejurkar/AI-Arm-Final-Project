# AI-Arm-Final-Project
Final year project


1. Problem Statement

 Warehouses often rely on manual labor for sorting, lifting, and placing parcels, which leads to:

 Higher labor costs

 Errors in sorting

 Slower operations

 Worker fatigue and injuries


2.Proposed Solution

An AI-powered robotic arm integrated with computer vision and automation systems to:

Detect parcels using a camera and AI model

Identify parcel size, shape, and barcode/label

Pick up the parcel using a robotic gripper

Place it in the correct section/bin based on category (e.g., weight, destination, type)


3.Technical Implementation

a.Hardware Components

  1.6-DOF robotic arm

  2.Servo Motors 

  3.Gripper (vacuum )

  4.Camera (RGB/Depth)

  5.Controller (Raspberry Pi)

b.Software & AI

  1.Computer Vision (OpenCV and Tensorflow detection)

  2.Barcode/QR code scanning

  3.Path planning (Inverse Kinematics, ROS MoveIt)

  4.AI/ML model for classification & sorting

  5.ROS (Robot Operating System) for control and integration

c.Workflow

  1.Camera captures parcel image

 2.AI model detects parcel features (size, shape, label, barcode)

 3.Decision-making module assigns a bin/section

4. Arm performs motion planning (pick → move → place)
5.Updates database/log for tracking



4.Advantages

  1.Speed & Efficiency → Continuous, no fatigue

  2.Accuracy → Correct sorting & tracking

  3.Scalability → Can be expanded for large warehouses

  4.Safety → Reduces human involvement in heavy-lifting


5.Example Use Cases

Amazon / Flipkart warehouses for parcel sorting


  



 
