sudo apt update
sudo apt-get install libboost-all-dev
sudo apt-get install cmake

git clone -b 4.1 https://github.com/borglab/gtsam.git
cd gtsam
mkdir build
cd build
cmake ..
sudo make install

echo "GTSAM successfully installed. Reboot now? (Y/n): "
read answer
if [["$answer" == "y" || "$answer" == "Y"] || -z "$answer"]; then
    echo "Rebooting now..."
    sudo reboot
else
    echo "Reboot canceled."
fi