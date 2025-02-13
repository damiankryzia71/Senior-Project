# first, download Armadillo 11.0.1 from
# https://sourceforge.net/projects/arma/files/armadillo-11.0.1.tar.xz/download?use_mirror=master
# then run this script

sudo apt update
sudo apt install cmake libopenblas-dev liblapack-dev libarpack2-dev libsuperlu-dev
cd ~
sudo mv Downloads/armadillo-11.0.1.tar.xz .
sudo tar xf armadillo-11.0.1.tar.xz
cd armadillo-11.0.1/
sudo cmake .
