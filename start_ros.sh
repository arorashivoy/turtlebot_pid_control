#!/bin/bash

run_in_terminal() {
    gnome-terminal -- bash -c "$1; exit"
}

launch_command="cd ~/colcon_ws; colcon build; source ~/colcon_ws/install/setup.bash; ros2 launch turtlebot3_gazebo empty_world.launch.py"

run_command="sleep 6; source ~/colcon_ws/install/setup.bash; ros2 run turtlebot_pid_control controller"

run_in_terminal "$launch_command"
run_in_terminal "$run_command"

echo "Done"
