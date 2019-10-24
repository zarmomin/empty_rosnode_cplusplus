#include <ros/ros.h>
#include <empty_rosnode_cplusplus/some_dummy_class.h>

/** Main node entry point. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "empty_rosnode_cplusplus");
  ros::NodeHandle node;
  ros::NodeHandle nh_private("~");
  ros::Publisher pub_odom_G_S, pub_odom_G_B;
  std::string filename, out_file_name;
  nh_private.param<std::string>("filename", filename, "");
  nh_private.param<std::string>("out_file_name", out_file_name, "");

  if (!filename.empty())
  {
    some_dummy_class sdc;
    sdc.transformKindrToValidRotationMatrix(filename, out_file_name);
  }
  else {
    ROS_INFO("this is pointless");
    ros::shutdown();
  }

  return 0;
}
