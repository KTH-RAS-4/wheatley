# Wheatley
The ROS package for the robot built by group 4 in the course DD2425 Robotics and Autonomous Systems at KTH in the fall of 2014.

#Installation
This package is intended for use with ROS hydro on Ubuntu 12.04.

To install, first run ```sudo -l > /dev/null ``` to log in as sa sudo user, and then run

```bash
cd /tmp #or wherever
wget https://raw.githubusercontent.com/KTH-RAS/ras_install/hydro-2014/scripts/install_nuc.sh
chmod +x install_nuc.sh
./install_nuc.sh
source ~/.bashrc

cd ~/catkin_ws/src
wget https://raw.githubusercontent.com/KTH-RAS/ras_install/hydro-2014/rosinstall/ras_utils.rosinstall
wstool merge ras_utils.rosinstall
wstool update

git clone https://github.com/KTH-RAS-4/wheatley.git
cd ~/catkin_ws
catkin_make executor_generate_messages occupancy_grid_utils_generate_messages ras_arduino_msgs_generate_messages ras_msgs_generate_messages sensors_generate_messages sound_play_generate_messages vision_msgs_generate_messages
catkin_make all
```

Running the robot can then be done with the following commands; in separate terminals.

```bash
roslaunch main base.launch
roslaunch main competition.launch phase:=1 save_map:=true
```

#packages
I a broad overview, the packages are:
##main
Launchfiles for bringing everything up.
##controllers
Motor controller, wall follower, executor, xbox...
##sensors
IR distance sensors, pose...
##mapping
Occupancy grid and object mapper.
##navigation
Pathfinder and path follower.
##vision
Object detection and recognition.
