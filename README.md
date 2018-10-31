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

## Setting Hokuyo
1. Install ROS package
  ```
  sudo apt-get install ros-kinetic-urg-node
  ```
2. Run Hokuyo node
  * USB port Hokuyo
    ```
    rosrun urg_node urg_node _serial_port:=<Hokuyo USB Port>
    ```
    Default port will be /dev/ttyACM0
  * Ethernet port Hokuyo
##


## Erros
ssh -X
rviz
rviz::RenderSystem: error
