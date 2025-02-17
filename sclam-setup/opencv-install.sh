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
