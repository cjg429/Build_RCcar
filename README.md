# Build_RCcar
How to make RCcar from the very beginning
## Install Ununtu on Jetson TX2
1. cd NVIDIA-INSTALLER
2. sudo ./installer.sh

The initial password of the Jetson TX2 is **nvidia**.
## Install ROS
1. Download sh file at https://github.com/jetsonhacks/installROSTX2
2. ./installROS.sh -p ros-kinetic-desktop-full
3. Build your catkin workspace and write <catkin_ws_dir>/devel/setup.bash on .bashrc


## Erros
ssh -X
rviz
rviz::RenderSystem: error
