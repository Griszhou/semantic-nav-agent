# Evaluation protocol

The replay is a deterministic smoke test, not a claim about physical-robot performance. It verifies the contracts needed for a larger benchmark:

| Metric | Definition |
| --- | --- |
| success | all requested semantic targets reached |
| path_length | number of traversed grid edges |
| SPL | optimal path length divided by max(actual, optimal) |
| replanning_ms | time spent in recovery and task-level replanning |
| explored_frontiers | number of frontier selections in the trace |

For ROS2 experiments, record a task manifest, rosbag2, map seed, detector model checksum, and Nav2 parameters. The evaluator should report mean and 95% bootstrap interval across seeds, plus failure categories (unknown target, blocked route, action timeout, and recovery exhaustion).

