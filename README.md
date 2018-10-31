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
  
  
##


## Erros
ssh -X
rviz
rviz::RenderSystem: error
