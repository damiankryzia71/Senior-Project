cd ~
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev g++ wget unzip
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
sudo wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
sudo wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip
sudo unzip opencv.zip
sudo unzip opencv_contrib.zip
sudo mkdir opencv-build
cd opencv-build
sudo cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules ../opencv-4.x
sudo make -j$(nproc)
sudo make install
