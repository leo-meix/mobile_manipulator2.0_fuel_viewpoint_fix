# Mobile Manipulator Autonomous Exploration

ROS Noetic workspace containing only the Gazebo mobile-manipulator autonomous
exploration pipeline and YOLO table segmentation.

## Runtime pipeline

1. Gazebo simulates the mecanum base, UR5 arm and end-effector D435 camera.
2. Camera depth is transformed into the world frame and fused into `sdf_map`.
3. FUEL-style frontier extraction selects reachable whole-body viewpoints.
4. REMANI plans and controls the base and arm trajectory.
5. YOLO segments tables from the D435 RGB stream and publishes annotated images
   and a binary table mask.

## Build

```bash
cd ~/eefixed_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

Python requirements for the detector are `ultralytics`, `opencv-python` and
`numpy`; ROS must provide `rospy`, `sensor_msgs` and `cv_bridge` in the same
Python environment.

## Run everything

```bash
roslaunch robot_gazebo robot1_remani_map.launch
```

YOLO is enabled by default in headless mode. Useful overrides:

```bash
roslaunch robot_gazebo robot1_remani_map.launch \
  enable_yolo:=true yolo_display:=false yolo_confidence:=0.3
```

Outputs:

- `/yolo_table_detector/annotated`: RGB image with masks and boxes
- `/yolo_table_detector/mask`: combined binary table mask
- REMANI/FUEL map, frontier, goal and trajectory visualization topics

The trained model is `src/yolo_table_detector/weights/best.pt`. Original REMANI
paper demos, fake simulators, random-map generators, grasping examples, unused
robot/camera variants and generated cache files were removed intentionally.
