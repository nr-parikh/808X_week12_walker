# Simple walker behavior on Turtlebot
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://github.com/nr-parikh/808X_week12_walker/blob/master/LICENSE)

The repository shows how to generate a simple exploring behavior on turtlebot. It uses laser scan messages being published to check if there is something in its vicinity. If there is then it rotates until it finds a free space and then goes again to explore the environment. 

## Dependencies 
The dependencies of this repository are:

```
* Ubuntu 16.04
* ROS Kinetic Kame
* Turtlebot Packages
```

Before proceedigng to install ROS, ensure that version of Ubuntu is 16.04. To install ROS follow the steps given below:

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full
```

After installation of ROS, the next step is to initialize and install its dependencies using following commands:

```
$ rosdep init
$ rosdep update 
```

The next step is to setup ROS environment which can be done using following commands:

```
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

Following the environment setup, next step is to create the workspace.

```
$ cd <path where workspace needs to be created>
$ mkdir -p <name of workspace>
$ cd <workspace>
<workspace>$ catkin_make
```

Just like the path of the ROS was sourced in *.bashrc* file, same needs to be done for the workspace by writing `source <path to workspace>/devel/setup.bash` at the end of the *.bashrc* file.
This avoids the need to source every time one needs to use workspace's packages.

Other dependency for this repository is *turtlebot* packages which allow the use of turtlebot in gazebo. Please follow the instructions given below to install the packages:
```
<home> sudo apt-get install ros-kinetic-turtlebot-*
```
The command above installs all turtlebot packages instead of cherry picking them. 

## Building the code

The code can be built by cloning the repository and executing following steps:
```
<home>$ cd <workspace>/src
<workspace>/src$ git clone https://github.com/nr-parikh/808X_week12_walker.git
<workspace>/src$ cd ..
<workspace>$ catkin_make 
```

## Running the code

After cloning the repository and building it, there are two ways to run the code using *rosrun* or *roslaunch* commands. The steps to run using both of them are given below.

### Running using *rosrun* commands
Please ensure that *rosmaster* is running before executing following steps. *rosmaster* can be started by following command.
```
<home>$ roscore
```

Before starting the node, it is necessary to launch the world in gazebo. This can be done as follows:
```
<home>$ roslaunch turtlebot_gazebo turtlebot_world.launch
```

To start the *turtlebot_walker* node follow the steps given below:
```
<home>$ rosrun 808X_week12_walker walker_node
```

### Running using *roslaunch* 
The launch file in this repository launches two things viz. turtlebot_world and the walker_node. 
```
<home>$ roslaunch 808X_week12_walker turtlebot_walker.launch 
```

Please note here that it is not mandatory to start *rosmaster* node while using *launch* file. It starts when the file is launched if it is not running. The command above will launch the node with default string in the launch file. 

### Recording *bag* file using the launch file

`turtlebot_walker` launch file of this package accepts a flag `record` that can be used to record the bag file which will record all the data that is being published. By default, the launch file will not record a *bag* file. It will save the file in `results` directory. This can be done using following commands:
```
<home>$ roslaunch 808X_week12_walker turtlebot_walker.launch record:=true
```

The launch file also accepts a flag which can be used to set how much duration should the bag file record. The flag is `duration`. The default value of this flag is *45* seconds however it can be changed as follows:
```
<home>$ roslaunch 808X_week12_walker turtlebot_walker.launch record:=true duration:=30
```

This command will record the bag file for *30* seconds.  

A sample bag file is included in the *results* directory. The *info* of this bag file which can be obtained using `rosbag info <../808X_week12_walker/results/turtlebot_walker.bag>` looks like below:
```
path:        results/turtlebot_walker.bag
version:     2.0
duration:    44.9s
start:       Dec 31 1969 19:00:00.29 (0.29)
end:         Dec 31 1969 19:00:45.17 (45.17)
size:        17.4 MB
messages:    33827
compression: none [23/23 chunks]
types:       bond/Status                           [eacc84bf5d65b6777d4c50f463dfb9c8]
             dynamic_reconfigure/Config            [958f16a05573709014982821e6822580]
             dynamic_reconfigure/ConfigDescription [757ce9d44ba8ddd801bb30bc456f946f]
             gazebo_msgs/LinkStates                [48c080191eb15c41858319b4d8a609c2]
             gazebo_msgs/ModelStates               [48c080191eb15c41858319b4d8a609c2]
             geometry_msgs/Twist                   [9f195f881246fdfa2798d1d3eebca84a]
             nav_msgs/Odometry                     [cd5e73d190d741a2f92e81eda573aca7]
             rosgraph_msgs/Clock                   [a9c97c1d230cfc112e270351a944ee47]
             rosgraph_msgs/Log                     [acffd30cd6b6de30f120938c17c593fb]
             sensor_msgs/Imu                       [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/JointState                [3066dcd76a6cfaef579bd0f34173e9fd]
             sensor_msgs/LaserScan                 [90c7ef2dc6895d81024acba2ac42f369]
             std_msgs/String                       [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage                    [94810edda583a504dfda3829e70d7eec]
topics:      /clock                                            4492 msgs    : rosgraph_msgs/Clock                  
             /cmd_vel_mux/active                                  1 msg     : std_msgs/String                      
             /cmd_vel_mux/input/navi                            448 msgs    : geometry_msgs/Twist                  
             /cmd_vel_mux/parameter_descriptions                  1 msg     : dynamic_reconfigure/ConfigDescription
             /cmd_vel_mux/parameter_updates                       1 msg     : dynamic_reconfigure/Config           
             /depthimage_to_laserscan/parameter_descriptions      1 msg     : dynamic_reconfigure/ConfigDescription
             /depthimage_to_laserscan/parameter_updates           1 msg     : dynamic_reconfigure/Config           
             /gazebo/link_states                               4486 msgs    : gazebo_msgs/LinkStates               
             /gazebo/model_states                              4486 msgs    : gazebo_msgs/ModelStates              
             /gazebo/parameter_descriptions                       1 msg     : dynamic_reconfigure/ConfigDescription
             /gazebo/parameter_updates                            1 msg     : dynamic_reconfigure/Config           
             /joint_states                                     4373 msgs    : sensor_msgs/JointState               
             /laserscan_nodelet_manager/bond                     88 msgs    : bond/Status                           (2 connections)
             /mobile_base/commands/velocity                     437 msgs    : geometry_msgs/Twist                  
             /mobile_base/sensors/imu_data                     4373 msgs    : sensor_msgs/Imu                      
             /mobile_base_nodelet_manager/bond                  176 msgs    : bond/Status                           (3 connections)
             /odom                                             4370 msgs    : nav_msgs/Odometry                    
             /rosout                                             48 msgs    : rosgraph_msgs/Log                     (9 connections)
             /rosout_agg                                         31 msgs    : rosgraph_msgs/Log                    
             /scan                                              433 msgs    : sensor_msgs/LaserScan                
             /tf                                               5578 msgs    : tf2_msgs/TFMessage                    (2 connections)
             /tf_static                                           1 msg     : tf2_msgs/TFMessage
```

>NOTE: The launch file doesn't record any data related to *camera*.

### Playback *bag* file
The provided sample bag file or any recorded bag file using the instructions provided above can be played by following the commands given below:
```
<home>$ cd <worskpace>/src/808X_week12_walker/results
<../results>$ rosbag play turtlebot_walker.bag
```
One will see the output similar to the sample output given below.
```
nrparikh@ubuntu:~/catkin_ws/src/808X_week12_walker$ rosbag play results/turtlebot_walker.bag 
WARNING: Catkin package name "808X_week12_walker" does not follow the naming conventions. It should start with a lower case letter and only contain lower case letters, digits, and underscores.
[ INFO] [1511244506.044792480]: Opening results/turtlebot_walker.bag

Waiting 0.2 seconds after advertising topics... done.

Hit space to toggle paused, or 's' to step.
 [RUNNING]  Bag Time:      2.890364   Duration: 2.600364 / 44.880000               44506.03 
 ```

One can use, *space* key to toggle between pause and play. Pressing *s* key will skip to the next nearest second value. 

While playing the bag file, one can *echo* a topic to see what is being published on the that topic and to analyze the behavior of the robot. To do this, please ensure that the bag file is being played. In other terminal execute the following command:
```
<home>$ rostopic echo <topic which needs to be checked>
```

For example, if one wants to check the topic `/cmd_vel_mux/input/navi` this can be one as follows after running the bag file:
```
<home>$ rostopic echo /cmd_vel_mux/input/navi
```

>NOTE: Please note that Gazebo should not be running while playing the bag file.