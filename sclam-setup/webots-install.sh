cd ~
wget https://github.com/cyberbotics/webots/releases/download/R2023a/webots_2023a_amd64.deb
sudo apt update
sudo apt install ./webots_2023a_amd64.deb

echo "Webots successfully installed. Reboot now? (Y/n): "
read answer
if [["$answer" == "y" || "$answer" == "Y"] || -z "$answer"]; then
    echo "Rebooting now..."
    sudo reboot
else
    echo "Reboot canceled."
fi