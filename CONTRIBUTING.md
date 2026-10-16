# Contributing

1. Configure with CMake and run `ctest --test-dir build --output-on-failure`.
2. Keep planner and Agent logic dependency-light; put ROS2-specific code under `ros2/`.
3. Add a regression test for every map, planning, or state-machine behavior change.
4. Do not commit bags, model weights, credentials, or generated build directories.
