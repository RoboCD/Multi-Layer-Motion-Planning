#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vgraph_environment");

    ros::NodeHandle nodeHandle;
    ros::Publisher publisher = nodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    visualization_msgs::Marker marker;

    while (ros::ok()){
    
        marker.header.frame_id = "/my_frame";
        marker.header.stamp = ros::Time();

        marker.ns = "test_square";
        marker.id = 0;

        // marker.type = visualization_msgs::Marker::CUBE;
        marker.type = visualization_msgs::Marker::LINE_LIST;

        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        // generate points
        float f = 0.0;
        for (uint32_t i = 0; i < 100; ++i)
        {
            float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
            float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

            geometry_msgs::Point p;
            p.x = (int32_t)i - 50;
            p.y = y;
            p.z = z;

            // points.points.push_back(p);
            // line_strip.points.push_back(p);

            // The line list needs two points for each line
            marker.points.push_back(p);
            p.z += 1.0;
            marker.points.push_back(p);
        }

        // sleep(10);
        while (publisher.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
            }
            ROS_INFO("Publishing marker");
            publisher.publish(marker);
    }

}

visualization_msgs::Marker init_marker( int marker_id,  uint32_t marker_type ){
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time();
    m.id = marker_id;
    m.ns = "ns1";
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;

    m.type = marker_type;
    m.scale.x = 0.01;
    m.scale.y = 0.01;
    m.color.g = 1.0;
    m.color.a = 1.0;
    return m;
}

std::vector<std::vector<std::pair<int, int>>> grow_obstacles ( std::vector<std::vector<std::pair<int, int>>> obstacles ){
    std::vector<std::vector<std::pair<int, int>>> grown_obstacles;

    int aabb_sidelen = 36;
    int half = aabb_sidelen / 2;

    std::pair<int, int> c1; 
    std::pair<int, int> c2;
    std::pair<int, int> c3;
    std::pair<int, int> c4;
    for (int o = 0; o < obstacles.size(); o++){

        for (int c = 0; c < obstacles[o].size(); c++){
            int coord1 = obstacles[o][c].first;
            int coord2 = obstacles[o][c].second;

            c1 = std::make_pair(coord1 - half, coord2 + half);
            c2 = std::make_pair(coord1 + half, coord2 + half);
            c3 = std::make_pair(coord1 - half, coord2 - half);
            c4 = std::make_pair(coord1 + half, coord2 - half);
           
        }
        grown_obstacles[o].push_back(c1);
        grown_obstacles[o].push_back(c2);
        grown_obstacles[o].push_back(c3);
        grown_obstacles[o].push_back(c4);
    }
    return grown_obstacles;
}