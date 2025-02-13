sudo apt update
sudo apt install cmake libopenblas-dev liblapack-dev libarpack2-dev libsuperlu-dev
sudo tar xf armadillo-11.0.1.tar.xz
cd armadillo-11.0.1/
cmake .

echo "Armadillo successfully installed. Reboot now? (Y/n): "
read answer
if [["$answer" == "y" || "$answer" == "Y"] || -z "$answer"]; then
    echo "Rebooting now..."
    sudo reboot
else
    echo "Reboot canceled."
fi