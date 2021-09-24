#include <ros/ros.h>
#include "flatland_map/flatland.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flatland");

  ros::NodeHandle nodeHandle("~");
  flatland_map::Flatland flatland(nodeHandle);

  ros::requestShutdown();
  return 0;
}