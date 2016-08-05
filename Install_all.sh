#!/bin/bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install rpi-update
sudo rpi-update

sudo apt-get install pi-bluetooth
sudo apt-get install bluez bluez-firmware
sudo apt-get install blueman
sudo usermod -G bluetooth -a pi

sudo apt-get install git

mkdir -p ~/src/
cd ~/src/
git clone --recursive git://github.com/thp/psmoveapi.git
cd psmoveapi
bash -e -x ./contrib/build_scripts/debian/build-ubuntu

cd 
cd ~/Downloads/
git clone --recursive https://github.com/Pete-22/PSMove_Websocket.git
git clone --recursive git://github.com/zaphoyd/websocketpp.git
git clone --recursive https://github.com/nlohmann/json.git
wget https://sourceforge.net/projects/boost/files/boost/1.61.0/boost_1_61_0.tar.gz/download
tar xvfz boost_1_61_0.tar.gz
sudo mv boost_1_61_0 /usr/local
cd ~/usr/local/boost_1_61_0
mkdir libbin
sudo ./bootstrap.sh --prefix=libbin
sudo ./b2
sudo ./b2 install

sudo apt-get install libboost-all-dev

echo "/usr/local/boost_1_47_0/libbin/lib" >> /etc/ld.so.conf.d/local.conf
echo "/usr/local/boost_1_61_0/libs \n /usr/local/boost_1_61_0/libbin \n /home/pi/src/psmoveapi/build" >> /etc/ld.so.conf.d/libc.conf
sudo ldconfig

cd 
cd ~/Downloads/json/src/
cp json.hpp ~/Downloads/PSMove_Websocket/
cp ~/src/psmoveapi/examples/c/psmove_examples_opengl.h ~/Downloads/PSMove_Websocket
cp ~/src/psmoveapi/include/psmove.h ~/Downloads/PSMove_Websocket
cp ~/src/psmoveapi/include/psmove_config.h ~/Downloads/PSMove_Websocket
cp ~/src/psmoveapi/include/psmove_fusion.h ~/Downloads/PSMove_Websocket
cp ~/src/psmoveapi/include/psmove_tracker.h ~/Downloads/PSMove_Websocket

cd ~/Downloads/PSMove_Websocket
make

sudo restart
