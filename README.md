# Build_RCcar
How to make RCcar from the very beginning
## Install Ununtu on Jetson TX2
1. ```cd NVIDIA-INSTALLER```
2. ```sudo ./installer.sh```

The initial password of the Jetson TX2 is **nvidia**.

## Download and install JetPack L4T
https://docs.nvidia.com/jetpack-l4t/2_2/content/developertools/mobile/jetpack/l4t/2.2/jetpack_l4t_install.htm
1. Download run file at https://developer.nvidia.com/embedded/jetpack on your host computer, not the Jetson board
2. ```chmod +x JetPack-${VERSION}.run```
3. ```./JetPack-${VERSION}.run```
4. Network Layout - Device accesses Internet via router/switch
5. Select the host nework by ```ifconfig```
6. USB Recovery Mode: When the Jetson board is powered off, press and hold REC button and then press PWR button
7. Your Jetson board will be powered on. Then connect to the same network to the host.

## Install ROS
1. Download sh files at https://github.com/jetsonhacks/installROSTX2
2. ```./installROS.sh -p ros-kinetic-desktop-full```
3. ```./setupCatkinWorkspace.sh ~./catkin_ws```

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
  3. Click **IPv4 Settings** and click **Manual** and click **Add** and set the new address like below
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
1. ```chmod +x librealsense.sh```
2. ```sudo ./librealsense.sh```
3. ```cd /usr/local/bin```
4. ```./realsense_viewer```
5. Install realsesne2_camera ROS package at https://github.com/intel-ros/realsense in your catkin_workspace
6. ```sudo apt-get install ros-kinetic-rgbd-launch```
7. ```roslaunch realsense2_camera rs_rgbd.launch```

## Network Settings
### On your Jetson board
1. Find **Network** on your toolbar and click **Edit Connections**
2. Click **Add** and select **Wi-Fi**
3. Click **Wi-Fi** and set the **Connection name** and **SSID**
```
Connection name      Hotspot_name  
SSID                 Hotspot_name
```
4. Add the following line to /etc/modprobe.d/bcmdhd.conf:
```
options bcmdhd op_mode=2
```
5. Click **Connect to Hidden Wi-Fi Network** and select the hotspot which you made.

After the Jetson board is connected to the hotspot, it cannot connect to any wi-fi network except for the hotspot.
If you want to connect other wi-fi network, you need to delete the hotspot network and the added line at step 4.

### On your notebook
1. Connect to the hotspot of the Jetson board
2. Click **Edit Connections** and select the hospot and click **Edit**
3. Click **IPv4 Settings** and click **Manual** and click **Add** and set the new address like below
```
IP Address      10.42.0.X(in this example, 64)
Subnet Mask     255.255.255.0
Default Gateway 10.42.0.1
```
4. Click **save**
5. Add the following line to /etc/hosts:
```
10.42.0.1 Name_of_the_Jetson_board(for example, nvidia)
```
6. Add the following lines to ~/.bashrc:
```
export ROS_MASTER_URI=http://10.42.0.1:11311
export ROS_IP=10.42.0.64
```
7. ```source ~/.bashrc```

### On your Jetson board
1. Add the following line to /etc/hosts:
```
10.42.0.64 Name_of_the_notebook(for example, rllab)
```
2. Add the following lines to ~/.bashrc:
```
export ROS_IP=10.42.0.1
export ROS_MASTER_URI=http://10.42.0.1:11311
```
3. ```source ~/.bashrc```

Now, you can run roscore on the Jetson board and it is connected to your notebook.

## Install ROS packages
1. ```sudo apt-get install ros-kinetic-navigation ros-kinetic-scan-tools ros-kinetic-hector-slam```
2. Download hwsetup folder in ~/catkin_ws/src
3. ```catkin_make```

## Errors
