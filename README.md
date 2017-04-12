[![Build Status](https://travis-ci.org/banuprathap/beginner_tutorials.svg?branch=master)](https://travis-ci.org/banuprathap/beginner_tutorials)

<a href="https://scan.coverity.com/projects/banuprathap-beginner_tutorials">
  <img alt="Coverity Scan Build Status"
       src="https://scan.coverity.com/projects/12074/badge.svg"/>
</a>

ROS Tutorial
============================
- ROS beginner tutorials for the course ENPM808X.

- This section covers tf and rosbag.

- This is a simple ROS package with three nodes.
1. talker
2. listener
3. modifier
- This package demonstrates how ROS communication is accomplished through *nodes* and *topics*. Each publishing *node* advertises the message to a *topic* that any *node* can subscribe to. This ensures that multiple nodes read the message seamlessly. This project can be extended to have multiple talkers and listeners. **Note:** each node should have a unique name.

Additionally, we have a *node* called modify which demonstrates the ROS services to alter the value of talker message.
 



## Dependencies

- ROS Indigo running on Ubuntu 14.04. 
- If you do not have ROS, download the bash script [here](https://gist.github.com/banuprathap/b2dab970df1f89573203b546c5eb3a5c) and run it as **sudo**. Note: This script assumes you're running Ubuntu 14.04.


### Package Dependency
- std_msgs
- roscpp
- rospy
- message_generation
- tf
- rosbag

## Build steps
- Open a terminal
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src && catkin_init_workspace
git clone --branch Week11_HW https://github.com/banuprathap/beginner_tutorials.git
cd ..
catkin_make
source ./devel/setup.bash
```

## Running the demo
- In your terminal
```bash
roslaunch beginner_tutorials demo.launch
```
Also, you can specify frequency and the message using **arg:=value**. For example to launch the talker at 100Hz to publish test use,

```bash
roslaunch beginner_tutorials demo.launch message:="test" frequency:=100
```
**Note**: For the specified message to take effect, open a new terminal and run,

```bash
rosrun beginner_tutorials modify
```
Now, this launch file will create a bag file named *mybag.bag* in your cloned directory. To make the listener printout the recorded info, open a terminal and run
```bash
rosrun beginner_tutorials listener
``` 
and in another terminal
```bash
rosbag play mybag.bag
```
##Inspecting bag files
To view information about the bag file that has been recorded by this demo
```
rosbag info mybag.bag
```
which will show something similar to 
```
path:        mybag.bag
version:     2.0
duration:    11.3s
start:       Apr 11 2017 22:43:13.96 (1491964993.96)
end:         Apr 11 2017 22:43:25.25 (1491965005.25)
size:        13.2 KB
messages:    114
compression: none [1/1 chunks]
types:       std_msgs/String [992ce8a1687cec8c8bd883ec73ca41d1]
topics:      /chatter   114 msgs    : std_msgs/String

```
##Inspecting tf frames
When the talker node is running,
```bash
rosrun tf view_frames
```
will create a file named **frames.pdf** which can be viewed by using
```bash
evince frames.pdf
```

Instead you can directly use the GUI application by simply typing
```
rqt &
```
To view the transform between *frame1* and *frame2* use,
```
 rosrun tf tf_echo frame1 frame2
```
 In our case, the command would be
```
 rosrun tf tf_echo world base
```

##Running Tests
To run the test cases,  in your catkin workspace
```bash
catkin_make test
```
This will trigger the CMake to run the test cases'

## Create Documentation

You will require **rosdoc_lite** package to create documents in ROS standards. You can get that by running
```bash
sudo apt-get install ros-indigo-rosdoc-lite
```
Once installed, run the following command replacing ***path*** with absolute path to the package in your file system. Assuming the *catkin_ws* is in your home directory, ***path*** would be *~/catkin_ws/src/beginner_tutorials/*

```bash
rosdoc_lite path
```