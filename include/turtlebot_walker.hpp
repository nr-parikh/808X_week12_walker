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
 * @file    turtlebot_walker.hpp
 * @author  nr-parikh
 * @copyright MIT license (c) 2017 Neel Parikh
 *
 * @brief DESCRIPTION
 * Class for generating walker behavior in turtlebot
 *
 */

#ifndef INCLUDE_TURTLEBOT_WALKER_HPP_
#define INCLUDE_TURTLEBOT_WALKER_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class turtlebotWalker {
 public:
  /**
   * @brief      Constructor of the class turtlebotWalker
   */
  turtlebotWalker();
  /**
   * @brief      Destructor of the class turtlebotWalker
   */
  ~turtlebotWalker();
  /**
   * @brief      Check if the collision happened from laser scan messages
   *
   * @param[in]  msg: The message from which collision has to be checked
   *
   * @return     void: Return nothing
   */
  auto collisionCheck(const sensor_msgs::LaserScan::ConstPtr& msg) -> void;
  /**
   * @brief      Explore the world until collision is detected
   */
  auto explore() -> void;

 private:
  bool collision_flag_;           ///< flag to check if the collision can happen
  ros::NodeHandle nh_;            ///< node handle for the class
  ros::Subscriber subscriber_;    ///< subscriber to get sensor messages
  ros::Publisher publisher_;      ///< publisher to publish velocity messages
  geometry_msgs::Twist vel_msg_;  ///< velocity message
};

#endif  // INCLUDE_TURTLEBOT_WALKER_HPP_
