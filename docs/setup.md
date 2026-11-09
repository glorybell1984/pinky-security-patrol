# Setup notes

- Every terminal: `source /opt/ros/jazzy/setup.bash` and the workspace `install/local_setup.bash`.
- Same `ROS_DOMAIN_ID` on the laptop and the robot. `ROS_LOCALHOST_ONLY` must be off for a real robot.
- Before each test: `ros2 node list` (no leftovers) and `fuser /dev/video0` (camera free).
- Record waypoints with RViz *Publish Point* and `ros2 topic echo /clicked_point --once`.
- Set the initial pose with *2D Pose Estimate* before patrolling.
