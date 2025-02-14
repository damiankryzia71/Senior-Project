cd ~
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
wget https://github.com/opencv/opencv/archive/4.2.0.zip
wget https://github.com/opencv/opencv_contrib/archive/refs/heads/4.x.zip
sudo unzip 4.2.0.zip
sudo unzip 4.x.zip
cd opencv-4.2.0
sudo mkdir build
cd build
sudo cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.x/modules ..
sudo make -j$(nproc)
sudo make install
