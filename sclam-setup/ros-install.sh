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

echo "ROS 2 Galactic successfully installed. Reboot now? (Y/n): "
read answer
if [["$answer" == "y" || "$answer" == "Y"] || -z "$answer"]; then
    echo "Rebooting now..."
    sudo reboot
else
    echo "Reboot canceled."
fi
