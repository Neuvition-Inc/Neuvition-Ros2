#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "driver.h"
int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("neuvition_node");

  rclcpp::Rate loop_rate(100);
  int count = 0;

  std_msgs::msg::String msg;
//  while (ros::ok())


  neuvition_driver::neuvitionDriver neu(node);


    neu.neuInit();

  while (rclcpp::ok())
  {
  //  std::stringstream ss;
  //  ss << "hello world " << count++;
  //  msg.data = ss.str();
//    ROS_INFO("%s", msg.data.c_str());
 ///  RCLCPP_INFO(node->get_logger(), "%s\n", msg.data.c_str());
//    ros::spinOnce();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  return 0;
}

