
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/ModString.h"

std::string g_str("default msg");


bool modify(beginner_tutorials::ModString::Request  &req,
            beginner_tutorials::ModString::Request  &res) {
  ROS_INFO("Service Call initiated");
  g_str = req.s;
  ROS_INFO("Message modification successful");
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::ServiceServer service = n.advertiseService("mod_string", modify);
  int rate(1);
  if (n.hasParam("f")) {
    ROS_INFO("Parameter f available");
    if (n.getParam("f", rate)) {
      ROS_INFO("Rewriting frequency \n");
    }
  }
  ros::Rate loop_rate(rate);
  unsigned int count = 0;
  std_msgs::String msg;
  std::stringstream ss;
  while (ros::ok()) {
    ss << g_str << "\t" << count << "\n";
    msg.data = ss.str();
    //  ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    ss.str("");
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
