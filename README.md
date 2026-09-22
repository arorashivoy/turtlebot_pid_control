# turtlebot_pid_control

**Archived.** A ROS 2 waypoint-following node for TurtleBot3, written December 2024
as IIIT-Delhi robotics coursework.

**The name is wrong and the repository is kept only for the date.** There is no
integral or derivative term anywhere in it. What it implements is proportional
control on heading alone: `k_w = 3` applied to the angle between the robot's yaw and
the bearing to the next waypoint, with error wrapped into (−π, π]. Linear velocity is
a hardcoded constant — a proportional term `k_v = 0.5` is computed and then discarded.
When the robot comes within 0.05 m of a waypoint it advances to the next one and the
list wraps around.

Pose comes from `/odom`, commands go to `/cmd_vel`, and the travelled path is
collected for a matplotlib plot at shutdown.

Superseded by [ros-maze-solver](https://github.com/arorashivoy/ros-maze-solver).
