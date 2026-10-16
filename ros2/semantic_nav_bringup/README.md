# ROS2/Nav2 bringup boundary

This folder is the integration contract for a ROS2 workspace. The standard-library C++ replay at the repository root remains the fastest way to validate planner and Agent logic.

In a ROS2 workspace, copy this package beside the core library and connect:

- `sensor_msgs/Image` + `CameraInfo` → RGB-D/YOLO detection adapter;
- `tf2_ros::Buffer` → `SemanticMap::integrate_detection` pose transform;
- `nav2_msgs/action/NavigateToPose` → navigation feedback and recovery trace;
- Gazebo world and Nav2 parameters → `config/nav2_params.yaml`.

