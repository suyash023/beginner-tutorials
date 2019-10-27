[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## ROS beginner tutorials

The code in this repository is based on the ros beginner tutorials found in its wiki from here http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem.

## Dependencies for code

The system runs on ROS Kinetic-kame with Ubuntu machine with packages roscpp, rospy, std_msgs installed.

## Building the code.
```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/suyash023/beginner_tutorials.git
cd ../..
catkin_make
```

## Running the code
Open 3 terminals. 

In terminal 1 start roscore.

In terminal 2 and 3 navigate to catkin_ws directory and type,

```
source ./devel/setup.bash
```
In terminal 2 type,
```
rosrun beginner_tutorials talker
```

In terminal 3 type,
```
rosrun beginner_tutorials listener
```


