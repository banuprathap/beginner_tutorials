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
 
#Status

- ~~Create a project called "beginner_tutorials" on GitHub before starting~~
- ~~Initialize your project "beginner_tutorials" as a git repository~~
- ~~Add the files to the repository as dictated by the tutorials~~
- ~~Commit them to the repo and make a descriptive commit message at the end of each tutorial~~
- ~~Modify the publisher node to publish a custom string message~~
- ~~Modify the tutorial code to follow Google C++ Style Guide~~
- ~~Run cpplint, fix issues (if any) and save output as a txt file to repository~~
- ~~Run cppcheck, fix issues (if any) and save output as a txt file to repository~~
- ~~Create a readme.md that provides an overview and build/run steps. Define assumptions/dependencies (e.g. ROS Indigo).~~
- ~~Push the final commits to GitHub once done.~~
Post to Canvas with your GitHub repository link.


#Dependencies

- ROS Indigo running on Ubuntu 14.04. 
- If you do not have ROS, download the bash script [here](https://gist.github.com/banuprathap/b2dab970df1f89573203b546c5eb3a5c) and run it as **sudo**. Note: This script assumes you're running Ubuntu 14.04.


###Package Dependency
- std_msgs
- roscpp

#Build steps
- Open a terminal
```bash
mkdir -p catkin_ws/src
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

#Create Documentation

You will require **rosdoc_lite** package to create documents in ROS standards. You can get that by running
```bash
sudo apt-get install ros-indigo-rosdoc-lite
```
Once installed, run the following command replacing ***path*** with absolute path to the package in your file system. Assuming the *catkin_ws* is in your home directory, ***path*** would be *~/catkin_ws/src/beginner_tutorials/*

```bash
rosdoc_lite path
```