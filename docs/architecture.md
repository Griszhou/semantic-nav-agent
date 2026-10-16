# Architecture notes

Semantic Nav Agent is split into a deterministic C++ core and an integration boundary. The core is intentionally dependency-light so a reviewer can build it without a robot, while the boundary documents where ROS2 messages and Nav2 actions enter the system.

## Data flow

1. A natural-language request is normalized into target labels and an ordered task list.
2. YOLO detections are fused with RGB-D depth and the robot pose. The map stores a confidence-weighted object pose and a geometry cell.
3. A known target goes directly to A*. An unknown target triggers frontier scoring. Utility combines information gain, semantic relevance, path cost, and risk.
4. The navigation adapter sends the path to Nav2. Feedback is recorded as a trace. A timeout or blocked path returns to recovery, where the Agent can choose another frontier or target.

The `TaskAgent` is a small state machine rather than a free-form chat loop. This keeps model actions bounded: tools can query the map and request plans, while only the navigation adapter can execute a goal.

## Integration boundary

`ros2/semantic_nav_bringup/` contains a package manifest, launch sketch, and Nav2 parameters. Those files are intentionally separate from the standard-library replay. In a ROS2 workspace, the adapter can translate `sensor_msgs`, `nav_msgs`, and `NavigateToPose` feedback to the core types without changing planner or evaluation logic.

