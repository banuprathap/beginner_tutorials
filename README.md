[![Build Status](https://travis-ci.org/banuprathap/beginner_tutorials.svg?branch=master)](https://travis-ci.org/banuprathap/beginner_tutorials)

<a href="https://scan.coverity.com/projects/banuprathap-beginner_tutorials">
  <img alt="Coverity Scan Build Status"
       src="https://scan.coverity.com/projects/12074/badge.svg"/>
</a>

ROS Publisher/Subscriber Tutorial
============================
- ROS beginner tutorials for the course ENPM808X.
- This is a simple ROS package with two nodes.
1. talker
2. listener
- This package demonstrates how ROS communication is accomplished through *nodes* and *topics*. Each publishing *node* advertises the message to a *topic* that any *node* can subscribe to. This ensures that multiple nodes read the message seamlessly. This project can be extended to have multiple talkers and listeners. **Note:** each node should have a unique name.
 

#Dependencies

- ROS Indigo running on Ubuntu 14.04


###Package Dependency
- std_msgs
- roscpp

#Build steps
- Open a terminal
```bash
mkdir -p ctakin_ws/src
cd catkin_ws/src && catkin_init_workspace
git clone https://github.com/banuprathap/beginner_tutorials.git
cd ..
catkin_make
source ./devel/setup.bash
```

#Running the demo
- In your terminal
```bash
roscore
```
- Open a new terminal 
```bash
rosrun beginner_tutorials talker 
```

- Open another terminal
```bash
rosrun beginner_tutorials listener
```