# runs inside the sim-gazebo container: NAME and MULTI_STAND_* from the environment
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
export MULTI_STAND_OUT=/cp/${NAME}_out
export MULTI_STAND_LAUNCH_ARGS="gui:=true ${MULTI_STAND_LAUNCH_ARGS:-}"
bash $(ros2 pkg prefix sim_gazebo)/lib/sim_gazebo/run_multi_stand_pass.sh
echo PASS_EXIT=$?
ls /cp/${NAME}_out | head -30
