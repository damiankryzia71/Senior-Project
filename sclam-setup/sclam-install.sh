sudo apt install ros-galactic-webots-ros2-driver

cd ~
git clone https://github.com/rodrigo-munguia/SCLAM-UAVs.git

# After cloning the repository open the file:
# ~/SCLAM-UAVs/src/globalslam/src/gmap_init_new_map_points.cpp
# and comment out the line: #include <numbers>
# save the file and continue with this script

cd ~/SCLAM-UAVs/
source /opt/ros/galactic/setup.bash
colcon build --packages-select interfaces
source install/setup.bash
colcon build

#launch the project with
./launch.sh
