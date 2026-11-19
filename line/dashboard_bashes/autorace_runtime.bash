# Source this file in each Autorace terminal. No driver/node is started.
# Use installed, verified versions; no pip/conda/OS packages are changed.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH PYTHONNOUSERSITE
export PATH="/usr/bin:/bin:$PATH"
source /opt/ros/humble/setup.bash
source /home/wego/wego_ws/install/local_setup.bash
export PYTHONPATH="/home/wego/miniconda3/envs/dl_env/lib/python3.10/site-packages:/usr/lib/python3.10/dist-packages:$PYTHONPATH"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=1
