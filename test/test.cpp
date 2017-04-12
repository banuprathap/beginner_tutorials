/*
 *MIT License
 *
 *  Copyright (c) 2017 Banuprathap Anandan
 *
 *  AUTHOR : BANUPRATHAP ANANDAN
 *  AFFILIATION : UNIVERSITY OF MARYLAND, MARYLAND ROBOTICS CENTER
 *  EMAIL : BPRATHAP@UMD.EDU
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *
 *
 *
 *  Project: ROS Beginner Tutorial
 *
 *
 */
/**
 * @file test.cpp
 * @brief A simple unit test for talker.cpp
 * @author Banuprathap Anandan
 * @date   03/14/2017
 */
#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/ModString.h"
std::shared_ptr<ros::NodeHandle> nh;
/**
 * @brief      test ModString server
 *
 */
TEST(TESTSuite, modstring) {
  ros::ServiceClient client = nh->serviceClient<beginner_tutorials::ModString>(
                                "mod_string");
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);
  beginner_tutorials::ModString srv;
  srv.request.s = "test";
  bool success(client.call(srv));
  EXPECT_TRUE(success);
}
/**
 * @brief      program entrypoint
 *
 * @param      argc  The argc
 * @param      argv  The argv
 *
 * @return     integer 0 upon exit success \n
 *            integer -1 upon exit failure
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "mod_string_client");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
