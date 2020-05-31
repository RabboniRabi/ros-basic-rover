# ros-basic-rover
###### Code for a basic three wheeled rover written to run using ROS


###### Scripts
The python scripts in this ROS package have been written as an exercise in understanding ROS and as a first project in robotics.

There are two sets of scripts for two modes of operating the rover.

For moving the rover with a keyboard, run the following nodes along wih roscore:

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


###### Rover

The three wheeled rover has a standard robot kit chasis to which two 3V micro-metal gear motors with wheels have been attached by cable tie. The direction of rotation of the two motors are controlled via a LN298N controller. A third wheel (castor wheel) is attached to the rear of the chasis.

The code runs on a Raspberry Pi 4 with ROS installed.

To keep the rover mobile, a power bank capable of powering the Raspberry Pi was used.

Two HC-SR04 ultrasonic sensors were placed in front of the rover for obstacle detection. Since the echo input from the echo pin of the ultrasonic sensor will be 5V and the Raspberry Pi can only handle 3.3V on its input pins, a voltage divider circuit was used for each sensor. 

###### Components

- [Chasis kit](https://robokits.co.in/robot-kits/robot-chassis-kit/mdf-robot-chassis-kit-with-tachometer-encoder-bo-motors)
- [Micro Metal Gear Motors](https://robokits.co.in/motors/metal-gear-micro-dc-motor/ga12-n20-3v-45-rpm-all-metal-gear-micro-dc-motor-with-precious-metal-brush)
- [LN298N Controller](https://robokits.co.in/motor-drives-drivers/dc-motor-driver/l298n-2a-dual-motor-driver-module-with-pwm-control)
- [3A power bank capable of powering RaspberryPi 4](https://www.boat-lifestyle.com/products/energyshrowm-pb10)
- [Raspberry Pi 4](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/)

###### Acknowledgements
The incredible amount of detailed information people have shared on the internet that enables anyone interested to learn, get a sound introduction to these topics.

[Article](https://thepihut.com/blogs/raspberry-pi-tutorials/hc-sr04-ultrasonic-range-sensor-on-the-raspberry-pi) from ThePiHut that helped in understanding how ultrasonic sensors work and how to use them.



