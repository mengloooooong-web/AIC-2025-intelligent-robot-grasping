source ./install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/Inuitive/InuDev/lib
ros2 launch install/inuros2/share/inuros2/launch/Inuros2.launch.py
