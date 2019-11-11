[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## ROS beginner tutorials

The code in this repository is based on the ros beginner tutorials found in its wiki from here http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem.

## Dependencies for code

The system runs on ROS Kinetic-kame with Ubuntu machine with packages roscpp, rospy, std_msgs, message_generation, tf installed.

The code also additionally uses the boost library

To build tests gtest and rostest are required

Additional rqt package components must also be installed to view the ros messages.

## Building the code.
```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/suyash023/beginner_tutorials.git
cd ../..
catkin_make
```

## Building and running the tests.
Go to the catkin_ws folder in a terminal and type:
```
catkin_make tests
```
To run and build tests you can also type:

```
catkin_make test
```

## Running the code

Source the environment of the catkin workspace.

```
source ./devel/setup.bash
```
Launch the talker_listener.launch file
```
roslaunch beginner_tutorials talker_listener.launch
```

Open a separate terminal and run 
```
rqt_console 
```
to view the ros messages being passed.

## Inspecting if tf frames are being published

To inspect if tf frames are working properly run the following command in a different terminal:
```
rosrun rqt_tf_tree rqt_tf_tree
```
A gui opens showing two frames /world and /talker. Check if they are visible and appropriate values are being published.

To save the tf tree in a pdf use the following command:
```
view_frames
```
## To record bag files

To record the data published on /chatter topic in a bag file pass an argument record:=true to the roslaunch command as follows
```
roslaunch beginner_tutorials talker_listener.launch record:=true
```
This creates a bag file in the ~/.ros/log folder.
to move the file use the following commnd with appropriate arguments
```
mv ~/.ros/log/(bag_file_name).bag (catkin_ws folder location)
```
## To play the bag files
To replay the recorded bag files use the following command:

First start roscore in a different terminal
```
roscore
```
Then run following command:
```
rosbag play results/2019-11-11-02-01-12.bag
```
Immediately in a following terminal start listener node using the following command
```
rosrun beginner_tutorials listener
```
You will see the messages captured being printed by the listener.

## Calling the ROS service

With launch file runnin in a terminal, open a new terminal. Navigate to the catkin_ws directory and source the setup.bash file as in the above steps.
Then run the following command

```
rosservice call /change_string Hello!
```
Expected response is the attachment of the time stamp to the string Hello! passed as an argument. The string can be changed.


