# Build_RCcar
How to make RCcar from the very beginning
## Install Ununtu on Jetson TX2
1. ```cd NVIDIA-INSTALLER```
2. ```sudo ./installer.sh```

The initial password of the Jetson TX2 is **nvidia**.

## Install ROS
1. Download sh files at https://github.com/jetsonhacks/installROSTX2
2. ```./installROS.sh -p ros-kinetic-desktop-full```
3. ```./setupCatkinWorkspace.sh ~/catkin_ws```

## Install ACM module to use Teensy and Hokuyo
1. Download sh file at https://github.com/jetsonhacks/installACMModule
2. ```sudo ./installCDCACM.sh```
3. Reboot the Jetson board

## Setting Hokuyo
1. Install ROS package
  ```
  sudo apt-get install ros-kinetic-urg-node
  ```
2. Run Hokuyo node
  * USB port Hokuyo(Default USB port: /dev/ttyACM0)
    ```
    rosrun urg_node urg_node _serial_port:=<Hokuyo USB Port>
    ```
  * Ethernet port Hokuyo
  1. Find **Network** on your toolbar and click **Edit Connections**
  2. Click **Add** and select **Ethernet**
  3. Click **IPv4 Settings** and click **Add** and set the new address like below
  ```
  Name            Hokuyo   
  IP Address      192.168.1.15
  Subnet Mask     255.255.255.0
  Default Gateway 192.168.1.1
  ```
  4. Connect Hokuyo ethernet port to the Jeston board and connect to **Hokuyo** at **Ethernet networks**
  5. ```rosrun urg_node urg_node _ip_address:=192.168.1.1```
  
## How to compile Teensy
You need to Install Arduino and Teensyduino to compile the Teensy board.
1. Install Arduino for Linux 64bit at https://www.arduino.cc/en/Main/Donate on your notebook
2. ```./install.sh```
3. Install Teensyduino at https://www.pjrc.com/teensy/td_download.html
4. ```chmod +x TeensyduinoInstall.linux64```
5. ```./TeensyduinoInstall.linux64``` 
 
   During the installation, you need to set your Teensyduino path at your extracted Arduino folder.
6. ```sudo apt-get install ros-kinetic-rosserial-arduino```
7. ```rosrun rosserial_arduino make_libraries.py <your Arduino path/libraries>```
8. 
```
sudo rm -f /tmp/49-teensy.rules /etc/udev/rules.d/49-teensy.rules &&
wget -O /tmp/49-teensy.rules https://www.pjrc.com/teensy/49-teensy.rules &&
sudo install -o root -g root -m 0664 /tmp/49-teensy.rules /lib/udev/rules.d/49-teensy.rules &&
sudo service udev reload && echo "Success."
```
9. ```sudo cp /tmp/49-teensy.rules /etc/udev/rules.d/49-teensy.rules```
10. Restart Arduino and reconnect your Teensy to the notebook
11. Compile and upload ~ on your Teensy

## How to use Teensy 
1. ```sudo apt-get install ros-kinetic-rosserial```
2. Install race ROS package at https://github.com/mlab-upenn/f1tenthpublic in your catkin workspace and make
3. ```rosrun rosserial_python serial_node.py <Teensy_USB_port>```

## Install RealSense Libary
You need to install RealSense library and realsense2_camera ROS package.
1. https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md
2. Install realsesne2_camera ROS package at https://github.com/intel-ros/realsense in your catkin_workspace
3. ```sudo apt-get install ros-kinetic-rgbd-launch```
4. ```roslaunch realsense2_camera rs_rgbd.launch```

## Erros
ssh -X
rviz
rviz::RenderSystem: error
