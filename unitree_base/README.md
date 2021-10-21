# unitree_base


# Dependencies

unitree github

- unitree_legged_sdk : https://github.com/unitreerobotics/unitree_legged_sdk


# Install & Build

## unitree_base
- ROS Melodic(PC) <-> ROS Kenitic(Robot) Tested 
- LCM Server (2ms loop)
- High Level Control

```bash
$ cd ~/catkin_ws
$ catkin_make
```

####Careful!!!

Build Order : unitree_legged_sdk -> unitree_base

You should to build 'unitree_legged_sdk' first then build 'unitree_base' in catkin_ws/src
Because CMakelists.txt in unitree_base make executable file from example code in unitree_legged_sdk

## Implementation

- Robot
```bash
($ ssh pi@192.168.123.12)
$ roscore
```

- PC (Wifi Connected with Robot)

```bash
[Terminal 1]

$ sudo su
(your password)
$ roslaunch unitree_base base.launch
```

```bash
[Terminal 2]

$ rosrun rqt_publisher rqt_publisher

publish /unitree_base/cmd_vel
```