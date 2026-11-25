source /opt/ros/jazzy/setup.bash
export MAKEFLAGS=-j1 CCACHE_DIR=/ccache CCACHE_MAXSIZE=6G
cd /ws
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH; source /opt/ros/jazzy/setup.bash
colcon build --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER_LAUNCHER=/cct/bin/ccache -DCMAKE_CXX_COMPILER_LAUNCHER=/cct/bin/ccache -DCMAKE_CXX_STANDARD=17 -Doctomap_DIR=/usr/share/octomap --packages-up-to sim_gazebo reachability stand_navigator ft_conditioner compliant_trajectory_controller ft_bridge_hardware coverage_planner manipulation_manager coverage_task_msgs mobile_manipulator_moveit_config 2>&1 | tail -6
echo BUILD_EXIT=${PIPESTATUS[0]}
