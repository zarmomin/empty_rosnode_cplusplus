#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

std::string odom_frame;
ros::Publisher pub_odom;

void handleOdometry(const nav_msgs::OdometryConstPtr &cameraOdom) {
  pub_odom.publish(cameraOdom);
}

/** Main node entry point. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "empty_rosnode_cplusplus");
  ros::NodeHandle node;
  ros::NodeHandle nh_private("~");

  nh_private.param<std::string>("odom_frame", odom_frame, "odom");

  ROS_INFO_STREAM("[" << ros::this_node::getName() << "] "
                      << "odom_frame: " << odom_frame);

  ros::Subscriber pcl_sub = node.subscribe("/some_topic", 10, handleOdometry);

  pub_odom = node.advertise<nav_msgs::Odometry>("/some_other_topic", 10, false);

  ros::spin();

  return 0;
}
