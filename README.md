# ros-basic-rover
###### Code for a basic three wheeled rover written using ROS


###### Scripts
The python scripts in this ROS package have been written as an exercise in understanding ROS and as a first project in robotics.

There are two sets of scripts for two modes of operating the rover.

For moving the rover with a keyboard run the following nodes along wiyh roscore:

```
listener.py
commander.py
driver_v1.py
key_teleop_slow.py
```

For letting the rover "roam" (more of a straight line with obstacle avoidance!), run the folllowing nodes along with roscore:

```
proximity_checker.py
roamer.py
driver_v2.py
```






