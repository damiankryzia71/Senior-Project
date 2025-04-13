# PX4, Gazebo, QGroundControl, and ORB-SLAM3 setup on Ubuntu 20

Before starting, install git.
```bash
sudo apt install git
```

### 1. Install PX4
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
```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```
Download QGroundControl [here](https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage).
Optionally:
```bash
cd ~/Desktop
mv ~/Downloads/QGroundControl.AppImage .
```
Reboot the system.
```bash
sudo reboot
```
Run QGroundControl.
```bash
cd ~/Desktop
chmod +x QGroundControl.AppImage
./QGroundControl.AppImage
```

### 3. Build PX4 with Gazebo simulation.
First, run QGroundControl as described in the previous step.
Then, run this command.
```bash
cd ~/Desktop/PX4-Autopilot
make px4_sitl gazebo-classic_typhoon_h480
```
Gazebo should open and QGroundControl should connect. The UAV is now ready to fly.

Reboot the system.
```bash
sudo reboot
```

### 4. Install Pangolin with all of its dependencies
```bash
sudo apt install libglew-dev
cd ~/Desktop
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
git checkout v0.6
git switch -c v0.6
./scripts/install_prerequisites.sh -m apt all
cmake -B build
cd build
sudo make -j$(nproc)
sudo make install
```

Reboot the system.
```bash
sudo reboot
```

### 5. Install ORB-SLAM3
First, clone the ORB-SLAM3 repository.
```bash
cd ~/Desktop
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
git checkout c++14_comp
```
Open the `CMakeLists.txt` file in the main directory and change this line to the version of your installed OpenCV. In my case, it's 4.2.0.
```cmake
find_package(OpenCV 4.4)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()
```
Then, build ORB-SLAM3.
```bash
chmod +x build.sh
./build.sh
```

Reboot the system.
```bash
sudo reboot
```

### 6. Download test datasets
```bash
cd ~/Desktop
mkdir -p Datasets/EuRoc
cd Datasets/EuRoc
wget -c http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
mkdir MH01
unzip MH_01_easy.zip -d MH01/
rm MH_01_easy.zip
```

### 7. Run ldconfig to Update Cache
```bash
sudo ldconfig
```

### 8. Run some examples
```bash
cd ~/Desktop/ORB_SLAM3

# Pick of them below that you want to run

# Mono
./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml ~/Desktop/Datasets/EuRoc/MH01 ./Examples/Monocular/EuRoC_TimeStamps/MH01.txt dataset-MH01_mono

# Mono + Inertial
./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml ~/Desktop/Datasets/EuRoc/MH01 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_monoi

# Stereo
./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml ~/Desktop/Datasets/EuRoc/MH01 ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereo

# Stereo + Inertial
./Examples/Stereo-Inertial/stereo_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/EuRoC.yaml ~/Desktop/Datasets/EuRoc/MH01 ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereoi
```
