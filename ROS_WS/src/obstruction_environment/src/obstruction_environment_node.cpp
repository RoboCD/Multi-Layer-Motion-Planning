#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>

// #include "flatland_map/flatland.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstruction_environment");

    ros::NodeHandle nodeHandle("~");
    ros::Publisher publisher = nodeHandle.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    grid_map_msgs::GridMap message;

    // Create grid map.
    grid_map::GridMap map({"obstacleMap"});
    map.setGeometry(grid_map::Length(48,48),1, grid_map::Position(0,0));
    map.setFrameId("map");

    ros::Duration longDuration(10.0);
    ros::Duration duration(1.0);
    grid_map::Position testPos(1,1);
    map.setTimestamp(ros::Time::now().toNSec());

    grid_map::GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    ROS_INFO("Message Published");
    longDuration.sleep();

    // for (int i = 0; i<20; i++){

    
    map.atPosition("obstacleMap",testPos) = 0.0;
    duration.sleep();
    map.atPosition("obstacleMap",testPos) = 1.0;

    duration.sleep();

    // Publish
    map.setTimestamp(ros::Time::now().toNSec());
    grid_map::GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    ROS_INFO("Message Published");
    duration.sleep();
    // }
    // ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    ros::requestShutdown();
    return 0;
}