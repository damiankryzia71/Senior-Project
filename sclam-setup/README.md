# Visual SCLAM Project Setup
### Project Authors: https://github.com/rodrigo-munguia/SCLAM-UAVs

The steps in this guide were executed on a freshly installed Ubuntu 20.04 system running on VMWare.
- [VMWare Download](https://livecsupomona-my.sharepoint.com/:u:/g/personal/rickramirez_cpp_edu/EauwsEtVC3ZIpcxP7PNTYyQBfk_FGKrOK8cS6195BIdhJQ?e=aO1hES)
- [Ubuntu 20.04 ISO Download](https://releases.ubuntu.com/focal/ubuntu-20.04.6-desktop-amd64.iso)

Before launching and using the SCLAM project, necessary dependencies must be installed.
These include:
- ROS 2 Galactic
- Armadillo 11.0.1
- OpenCV 4.2.0 with contrib module, VTK, and Ceres
- GTSAM 4.1.1
- Webots v.R2023a

This guide will walk you through the installation of these dependencies and the main project.
The dependencies can be installed in any order, following the order of this depository is not required.

### 1. Install ROS 2
Run the following script to install ROS 2 Galactic and its necessary dependencies.

```bash
sudo apt update && sudo apt install locales
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-galactic-desktop
sudo apt install ros-galactic-ros-base
sudo apt install ros-dev-tools
```

Optionally: reboot the system to ensure everything is correct.
```bash
sudo reboot
```

### 2. Install Armadillo 11.0.1
First, download Armadillo 11.0.1 [here](https://sourceforge.net/projects/arma/files/armadillo-11.0.1.tar.xz/download?use_mirror=master).
Then run the script below to install it.

```bash
sudo apt update
sudo apt install cmake libopenblas-dev liblapack-dev libarpack2-dev libsuperlu-dev
cd ~
sudo mv Downloads/armadillo-11.0.1.tar.xz .
sudo tar xf armadillo-11.0.1.tar.xz
cd armadillo-11.0.1/
sudo cmake .
```

Optionally: reboot the system to ensure everything is correct.
```bash
sudo reboot
```

### 3. Install GTSAM 4.1.1
Run the following script to install GTSAM 4.1.1 and its necessary dependencies.

```bash
cd ~
sudo apt update
sudo apt-get install libboost-all-dev
sudo apt-get install cmake
git clone -b 4.1 https://github.com/borglab/gtsam.git
cd gtsam
sudo mkdir build && cd build
sudo cmake ..
sudo make install
```

Optionally: reboot the system to ensure everything is correct.
```bash
sudo reboot
```

### 4. Install OpenCV 4.2.0 with Contrib Modules
Run the following script to install OpenCV 4.2.0 and its necessary dependencies.

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential cmake git libgtk-3-dev libvtk7-dev \
                    libboost-all-dev libgflags-dev libgoogle-glog-dev \
                    libhdf5-dev libatlas-base-dev libeigen3-dev \
                    libtbb2 libtbb-dev libdc1394-22-dev libjpeg-dev \
                    libpng-dev libtiff-dev libavcodec-dev libavformat-dev \
                    libswscale-dev libopenexr-dev libxvidcore-dev \
                    libx264-dev libgtk-3-dev libgphoto2-dev \
                    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
                    libprotobuf-dev protobuf-compiler python3-dev \
                    python3-numpy python3-pip

sudo apt install -y libvtk7-dev
sudo apt install -y libceres-dev

cd ~
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
cd opencv
git checkout 4.2.0
cd ../opencv_contrib
git checkout 4.2.0

cd ~/opencv
sudo mkdir build && cd build
sudo cmake -D CMAKE_BUILD_TYPE=Release \
         -D CMAKE_INSTALL_PREFIX=/usr/local \
         -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
         -D WITH_VTK=ON \
         -D BUILD_opencv_viz=ON \
         -D BUILD_opencv_sfm=ON \
         -D WITH_CERES=ON \
         ..
sudo make -j$(nproc)
sudo make install
sudo ldconfig
```

Optionally: reboot the system to ensure everything is correct.
```bash
sudo reboot
```

### 5. Install Webots v.R2023a
Run the following script to install the Webots Simulator (v.R2023a).

```bash
cd ~
wget https://github.com/cyberbotics/webots/releases/download/R2023a/webots_2023a_amd64.deb
sudo apt update
sudo apt install ./webots_2023a_amd64.deb
```

Optionally: reboot the system to ensure everything is correct.
```bash
sudo reboot
```

### 6. Build the Visual SCLAM project.
First, install the webots_ros2_driver package for ROS 2.
```bash
sudo apt install ros-galactic-webots-ros2-driver
```
Then clone the project repository.
```bash
cd ~
git clone https://github.com/rodrigo-munguia/SCLAM-UAVs.git
```
After cloning the repository, open the file located at ~/SCLAM-UAVs/src/globalslam/src/gmap_init_new_map_points.cpp and comment out this line (the **numbers** library is not supported by Ubuntu 20's GCC/G++ 9.4.0 and other parts of the project seem to break after updating GCC/G++. At the same time, the file doesn't actually use this library anywhere.)
```cpp
#include <numbers>
```
Save the file.
Run the following script to build the project.
```bash
cd ~/SCLAM-UAVs/
source /opt/ros/galactic/setup.bash
colcon build --packages-select interfaces
source install/setup.bash
colcon build
```
Finally, launch the project by running the launch.sh script in the ~/SCLAM-UAVs/ directory.
```bash
./launch.sh
```
