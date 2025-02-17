cd ~
sudo apt update
sudo apt-get install libboost-all-dev
sudo apt-get install cmake
git clone -b 4.1 https://github.com/borglab/gtsam.git
cd gtsam
sudo mkdir build && cd build
sudo cmake ..
sudo make install
