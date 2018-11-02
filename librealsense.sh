#!/bin/bash
LIBREALSENSE_DIRECTORY=$HOME/librealsense
INSTALL_DIR=$PWD
echo "${blue}Making Ubuntu Up-to-date${blue}"
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
echo "${blue}Installing required libraries${blue}"
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev
cd $HOME
git clone https://github.com/IntelRealSense/librealsense.git
echo "${green}Successfully clone librealsense${green}"
cd librealsense
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/ 
sudo udevadm control --reload-rules && udevadm trigger
./scripts/patch-realsense-ubuntu-lts.sh
echo 'hid_sensor_custom' | sudo tee -a /etc/modules
mkdir build && cd build
echo "${green}Configuring Make system${reset}"
cmake ../
echo "${green}librealsense make successful${green}"
sudo make uninstall && make clean && make && sudo make install
sudo make install
echo "${green}Library Installed${reset}"
