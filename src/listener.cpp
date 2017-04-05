
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/ModString.h"


void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: %s", msg->data.c_str());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
 /*
  ros::ServiceClient client =
    n.serviceClient<beginner_tutorials::ModString>("mod_string");
  beginner_tutorials::ModString srv;
  if (n.hasParam("msg")) {
    if (!n.getParam("msg", srv.request.s)) {
      ROS_INFO("Error getting message \n");
      return 1;
    }
  }
  if (client.call(srv)) {
    ROS_INFO("Successfully modified");
  } else {
    ROS_ERROR("Failed to call service mod_string");
    return 1;
  }
 */
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();
  return 0;
}

