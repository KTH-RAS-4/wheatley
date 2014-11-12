# Wheatley
The main ROS package for the robot built by group 4 of KTH-RAS fall 2014.

#sub-packages

##controllers
Motor controllers, wall following...

##sensors
Distance sensors, current pose...

##vision
Object detection and recognition.
###image_publisher
Publishes the image stream from an attached camera on /camera/rgb/image_raw (same as the primesense). Run it with

```bash
rosrun image_publisher image_publisher_node $0
```

where `$0` is an integer defining which camera to use (try values from 0 and up).