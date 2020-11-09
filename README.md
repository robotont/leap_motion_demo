# leap\_motion\_demo
ROS package for demonstrating the synergy between the Leap Motion Controller and the Robotont robot.

Start Leap motion core services in robotont with
```bash
sudo leapd
```

After that you can run default demo as 
```bash
roslaunch leap_motion_demo demo.launch
```

Or demo with additional gestures:
```bash
roslaunch leap_motion_demo demo_with_gestures.launch
```

Robot will turn, if you put your fingers in your fist and move, if you pitch the index finger and thumb together.

Package send information directly to cmd_vel topic. 


