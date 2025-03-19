# PX4, Gazebo, QGroundControl, and ORB-SLAM3 setup on native installation of Ubuntu 22

#### Before starting, install git with:
```bash
sudo apt install git
```

### 1. Install PX4
Clone the repository and install dependencies.
```bash
cd ~/Desktop
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
Reboot the system.
```bash
sudo reboot
```

### 2. Install QGroundControl
Install dependencies.
```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```
Download QGroundControl Daily Build [here](https://d176tv9ibo4jno.cloudfront.net/builds/master/QGroundControl-x86_64.AppImage).
Optionally:
```bash
cd ~/Desktop
mv ~/Downloads/QGroundControl-x86_64.AppImage .
```
Reboot the system.
```bash
sudo reboot
```
Run QGroundControl.
```bash
cd ~/Desktop
chmod +x QGroundControl-x86_64.AppImage
./QGroundControl-x86_64.AppImage
```

### 3. Build PX4 with Gazebo simulation.
First, run QGroundControl as described in the previous step.
Then, run this command.
```bash
cd ~/Desktop/PX4-Autopilot
make px4_sitl gz_x500
```
Gazebo should open and QGroundControl should connect. The UAV is now ready to fly.

Reboot the system.
```bash
sudo reboot
```

### 4. Install Pangolin with all of its dependencies
```bash
cd ~/Desktop
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
git checkout v0.9.3
git switch -c v0.9.3
./scripts/install_prerequisites.sh -m apt all
mkdir build
cd build
cmake .. -D CMAKE_BUILD_TYPE=Release
sudo make -j$(nproc)
sudo make install
```

Reboot the system.
```bash
sudo reboot
```

### 5. Install ORB-SLAM3