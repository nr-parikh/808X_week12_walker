// MIT License

// Copyright (c) 2017 Neel Parikh

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * @file    turtlebot_walker.cpp
 * @author  nr-parikh
 * @copyright MIT license (c) 2017 Neel Parikh
 *
 * @brief DESCRIPTION
 * Class implementation for the class turtlebotWalker.
 *
 */

#include "turtlebot_walker.hpp"

turtlebotWalker::turtlebotWalker() {
  // Inform initialization process
  ROS_INFO_STREAM("Initializing publishers and subscribers..");
  // Publish velocity commands on /mobile_base/commands/velocity
  publisher_ =
      nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);
  // Subscribe to laser scan messages
  subscriber_ = nh_.subscribe<sensor_msgs::LaserScan>(
      "/scan", 1000, &turtlebotWalker::collisionCheck, this);

  // Initialize the flag to false
  collision_flag_ = false;
}

turtlebotWalker::~turtlebotWalker() {
  // Reset the velocity while destructing the object
  vel_msg_.linear.x = 0;
  vel_msg_.angular.z = 0;
  // Publish the velocity
  publisher_.publish(vel_msg_);
}

auto turtlebotWalker::collisionCheck(
    const sensor_msgs::LaserScan::ConstPtr& msg) -> void {
  // Set the flag to false by default
  collision_flag_ = false;
  // Iterate over the ranges
  for (auto i : msg->ranges) {
    // Check if the distance is less than threshold
    if (i < 1) {
      // Set the collision flag to true
      collision_flag_ = true;
    }
  }
}

auto turtlebotWalker::explore() -> void {
  // Initialize the rate at which the messages will be published
  ros::Rate rate(10.0);
  // Inform the process being done
  ROS_INFO_STREAM("Exploring the environment...");
  while (ros::ok()) {
    // If collision_flag_ is not set go forward
    if (!collision_flag_) {
      vel_msg_.linear.x = 0.2;
      vel_msg_.angular.z = 0.0;
      // Else rotate at the same place
    } else {
      vel_msg_.linear.x = 0.0;
      vel_msg_.angular.z = -0.1;
    }
    // Publish the velocity
    publisher_.publish(vel_msg_);
    // Spin in callbacks
    ros::spinOnce();
    // Sleep
    rate.sleep();
  }
}
