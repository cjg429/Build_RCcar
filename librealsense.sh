LIBREALSENSE_DIRECTORY=${HOME}/librealsense
INSTALL_DIR=$PWD
echo "Making Ubuntu Up-to-date"
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
echo "Installing required libraries"
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/ 
sudo udevadm control --reload-rules && udevadm trigger
cd ${HOME}
git clone https://github.com/IntelRealSense/librealsense.git
echo "{green}Successfully clone librealsense{green}"
cd $LIBREALSENSE_DIRECTORY
./scripts/patch-realsense-ubuntu-lts.sh
echo 'hid_sensor_custom' | sudo tee -a /etc/modules
mkdir build && cd build
echo "${green}Configuring Make system${reset}"
cmake ../
echo "{green}librealsense make successful{green}"
sudo make uninstall && make clean && make && sudo make install
sudo make install
echo "${green}Library Installed${reset}"
