#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "zr300_node");

  nodelet::Loader manager(true);
  nodelet::V_string nargv;

  manager.load(ros::this_node::getName(), "realsense_camera/ZR300Nodelet",
               ros::names::getRemappings(), nargv);

  ros::spin();
  return 0;
}
