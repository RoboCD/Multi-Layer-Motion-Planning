
#pragma once

#include <grid_map_ros/grid_map_ros.hpp>

// ROS
#include <ros/ros.h>

namespace flatland_map
{

    /*!
     * Visualizes a grid map by publishing different topics that can be viewed in Rviz.
     */
    class Flatland
    {
    public:
        /*!
         * Constructor.
         * @param nodeHandle the ROS node handle.
         */
        Flatland(ros::NodeHandle &nodeHandle);

        /*!
         * Destructor.
         */
        virtual ~Flatland();

        /*!
         * Publish the grid map to ROS.
         */
        void publish();

        // Fill an obstacle field until it meets the given coverage percent
        void obstacle_field(int cov);
        
        // Count the total coverage in the grid, both 1s and 2s
        int totalCoverage();

        // Check that a tetromino will fit in a given position
        bool checkTetGridPos(double tet[][2], int x, int y);

        // Insert the given tetromino at the given position in the grid
        void insert_tetromino(double tet[][2], int x, int y);

        // Implement breadth first search and return the number of iterations
        int breadthFirstSearch();

        // Implement depth first search and return the number of iterations
        int depthFirstSearch();

        // Implement dijkstra search and return the number of iterations
        int dijkstraSearch();

        // Implement random search and return the number of iterations
        int randomSearch();

        // Check if position on given map is available and valid. Return true if valid
        bool checkOpenPos(int x, int y, std::string searchMap);

        // Get the minimum distance from the list of distances
        int getMinDistance(double dist[],std::list<int> queue);

        // Get the length between two positions
        double getLength(grid_map::Position pos1, grid_map::Position pos2);
    private:
        //! ROS nodehandle.
        ros::NodeHandle &nodeHandle_;

        //! Grid map publisher.
        ros::Publisher gridMapPublisher_;

        //! Polygon publisher.
        ros::Publisher polygonPublisher_;

        //! Grid map data.
        grid_map::GridMap map_;
    };

} /* namespace */
