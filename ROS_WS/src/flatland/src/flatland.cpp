#include "flatland_map/flatland.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <random>
#include <list>
#include <algorithm>

// ROS
#include <geometry_msgs/PolygonStamped.h>

using namespace std;
using namespace ros;
using namespace grid_map;

namespace flatland_map
{

    double tetrominoT[3][2] = { { .5, 1 },
                             { 1, 1 },
                             { .5, 1 } };
    double tetrominoL[3][2] = { { 1, 1 },
                             { .5, 1 },
                             { .5, 1 } };
    double tetrominoSkew[3][2] = { { 1, .5 },
                                { 1, 1 },
                                { .5, 1 } };
    Flatland::Flatland(ros::NodeHandle &nodeHandle)
        : nodeHandle_(nodeHandle),
          map_(vector<string>({"obstacle","breadthSearch","depthSearch","dijkstraSearch","randomSearch"}))
    {
        ROS_INFO("Grid map iterators demo node started.");

        gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
        polygonPublisher_ = nodeHandle_.advertise<geometry_msgs::PolygonStamped>("polygon", 1, true);

        // Setting up map.
        double gridResolution = 1;
        double gridLength = 128;
        double resolutionLength = gridLength * gridResolution;
        map_.setGeometry(Length(resolutionLength, resolutionLength), gridResolution, Position(0.5, 0.5));
        map_.setFrameId("map");

        publish();
        ros::Duration duration(2.0);
        duration.sleep();
        map_.clearAll();
        publish();
        grid_map::Matrix &data = map_["obstacle"];
        

        Position startPosition (64,64);
        Position endPosition (-63,-63);
        map_.atPosition("obstacle",startPosition) = 0;
        map_.atPosition("obstacle",endPosition) = 0;

        publish();

        obstacle_field(70);
        duration.sleep();

        // Searches
        int breadthIts = breadthFirstSearch();
        ROS_INFO("Breadth Search Iterations %d", breadthIts);
        duration.sleep();

        int depthIts = depthFirstSearch();
        ROS_INFO("Depth Search Iterations %d", depthIts);
        duration.sleep();

        int dijkstraInt = dijkstraSearch();
        ROS_INFO("Dijkstra Search Iterations %d", dijkstraInt);
        duration.sleep();

        int randIts = randomSearch();
        ROS_INFO("Random Search Iterations %d", randIts);
        duration.sleep();
        publish();

    }

    Flatland::~Flatland() {}

   
    void Flatland::publish()
    {
        map_.setTimestamp(ros::Time::now().toNSec());
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(map_, message);
        gridMapPublisher_.publish(message);
        ROS_DEBUG("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
    }

    void Flatland::obstacle_field(int cov){
        ROS_INFO("Creating grid map of %d coverage", cov);
        int obstacleCov = (128*128)*cov/100;
        ROS_INFO("ObstacleCov %d", obstacleCov);
        int obstacleCount = 0;
        int currentCov = 0;
        ros::Duration duration(0.001);
        std::srand(5323);
        std::rand();
        currentCov = totalCoverage();
        ROS_INFO("Current Coverage %d", currentCov);
        while (currentCov < obstacleCov)
        {
            currentCov = totalCoverage();
            ROS_INFO("Current Coverage %d", currentCov);
            
            int randx;
            int randy;

            int t = std::rand() % 3 + 1;
            std::random_device rd;
            std::mt19937 mt(rd());
            std::uniform_int_distribution<int> dist(1,3);

            bool validPos = false;
            switch(t)
            {
                case 1:
                    validPos = false;
                    while (validPos == false){
                        randx = std::rand() % 126;
                        randy = std::rand() % 127;

                        validPos = checkTetGridPos(tetrominoT, randx, randy );
                    }
                    insert_tetromino(tetrominoT, randx, randy);
                case 2:
                    validPos = false;
                    while (validPos == false){
                        randx = std::rand() % 126;
                        randy = std::rand() % 127;
                        validPos = checkTetGridPos(tetrominoL, randx, randy );
                    }
                    insert_tetromino(tetrominoL, randx, randy);
                case 3:
                    validPos = false;
                    while (validPos == false){
                        randx = std::rand() % 126;
                        randy = std::rand() % 127;
                        validPos = checkTetGridPos(tetrominoSkew, randx, randy );
                    }
                    insert_tetromino(tetrominoSkew, randx, randy);
                default:
                    validPos = false;
                    while (validPos == false){
                        randx = std::rand() % 126;
                        randy = std::rand() % 127;
                        validPos = checkTetGridPos(tetrominoT, randx, randy );
                    }
                    insert_tetromino(tetrominoT, randx, randy);
            }
        }
        publish();
    }

    int Flatland::totalCoverage(){
        grid_map::Matrix &data = map_["obstacle"];
        int count = 0;
        for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
        {
            const int i = iterator.getLinearIndex();
            Position currentPos;
            map_.getPosition(*iterator, currentPos );
            if (( data(i) == 1.0)||( data(i) == 0.5)){
                count ++;
            }
        }
        return count;
    }

    bool Flatland::checkTetGridPos (double tet[3][2], int x, int y){
        grid_map::Matrix &data = map_["obstacle"];
        Index submapStartIndex(x, y);
        Index submapBufferSize(3, 2);
        int i = 0;
        for (grid_map::SubmapIterator iterator(map_, submapStartIndex, submapBufferSize);
            !iterator.isPastEnd(); ++iterator)
        {
            if (( map_.at("obstacle", *iterator) == 1.0)||
                ( map_.at("obstacle", *iterator) == 0.5)||
                ( map_.at("obstacle", *iterator) == 0.0)){
                return false;
            }
            i++;
        }
        return true;
    }

    void Flatland::insert_tetromino( double tet[][2], int x, int y){
        grid_map::Matrix &data = map_["obstacle"];

        Index submapStartIndex(x, y);
        Index submapBufferSize(3, 2);
        int i = 0;
        ros::Duration duration(0.001);
        for (grid_map::SubmapIterator iterator(map_, submapStartIndex, submapBufferSize);
            !iterator.isPastEnd(); ++iterator)
        {

            int tet_y = (i)/2;
            int tet_x = (i)%2;
            map_.at("obstacle", *iterator) = tet[tet_y][tet_x];
            i++;
        }
        duration.sleep();
        publish();
    }


    // ********************** Search Algorithms **********************

    int Flatland::breadthFirstSearch(){
        map_.add("breadthSearch");
        ros::Duration duration(0.01);
        publish();
        ROS_INFO("Breadth");
        Position startPosition (64,64);
        Position endPosition (-63,-63);
        
        Position currentPosition = startPosition;
        int currentXPos = currentPosition.x();
        int currentYPos = currentPosition.y();
        ROS_INFO("Breadth: CurrPos %d %d", currentXPos, currentYPos);

        list<Index> queue;

        Position getPos;
        Index i(0,0);
        map_.getPosition(i,getPos);

        queue.push_back(i);
        map_.atPosition("breadthSearch",currentPosition)= 1.0;
        duration.sleep();
        publish();
        int iterations = 0;
        while(!queue.empty()){
            i = queue.front();
            queue.pop_front();
            map_.getPosition(i,currentPosition);
            
            currentXPos = currentPosition.x();
            currentYPos = currentPosition.y();
            ROS_INFO("Breadth: CurrPos %d %d", currentXPos, currentYPos);
            if (currentPosition == endPosition){
                return iterations;
            }
            int subXPos = currentXPos - 1;
            int subYPos = currentYPos - 1;
            for (int x = 0; x<3; x++){
                for (int y = 0; y<3; y++){
                    bool validPos = checkOpenPos(subXPos+x,subYPos+y,"breadthSearch");
                    if (validPos == true){
                        int mapsubXPos = subXPos+x;
                        int mapsubYPos = subYPos+y;

                        ROS_INFO("Breadth: visit %d %d", mapsubXPos, mapsubYPos);
                        
                        Position subPos (mapsubXPos,mapsubYPos);
                        map_.atPosition("breadthSearch", subPos) = 1.0;
                        duration.sleep();
                        publish();
                        Index sIndex;
                        map_.getIndex(subPos,sIndex);
                        queue.push_back(sIndex);
                    }
                }
            }
        iterations++;
        }
        return iterations;
        publish();
    }

    int Flatland::depthFirstSearch(){
        map_.add("depthSearch");
        ros::Duration duration(0.01);
        publish();
        ROS_INFO("Depth");
        Position startPosition (64,64);
        Position endPosition (-63,-63);
        
        Position currentPosition = startPosition;
        int currentXPos = currentPosition.x();
        int currentYPos = currentPosition.y();
        ROS_INFO("Depth: CurrPos %d %d", currentXPos, currentYPos);
        list<Index> queue;

        Position getPos;
        Index i(0,0);
        map_.getPosition(i,getPos);

        queue.push_front(i);

        map_.atPosition("depthSearch",currentPosition)= 1.0;
        duration.sleep();
        publish();

        int iterations = 0;
        while(!queue.empty()){
            i = queue.front();
            queue.pop_front();
            map_.getPosition(i,currentPosition);
            
            currentXPos = currentPosition.x();
            currentYPos = currentPosition.y();
            ROS_INFO("Depth: CurrPos %d %d", currentXPos, currentYPos);
            if (currentPosition == endPosition){
                return iterations;
            }
            int subXPos = currentXPos - 1;
            int subYPos = currentYPos - 1;
            for (int x = 0; x<3; x++){
                for (int y = 0; y<3; y++){
                    bool validPos = checkOpenPos(subXPos+x,subYPos+y,"depthSearch");
                    if (validPos == true){
                        int mapsubXPos = subXPos+x;
                        int mapsubYPos = subYPos+y;

                        ROS_INFO("Depth: visit %d %d", mapsubXPos, mapsubYPos);
                        
                        Position subPos (mapsubXPos,mapsubYPos);
                        map_.atPosition("depthSearch", subPos) = 1.0;
                        duration.sleep();
                        publish();
                        Index sIndex;
                        map_.getIndex(subPos,sIndex);
                        queue.push_front(sIndex);
                    }
                }
            }
        iterations++;
        }
        return iterations;
        publish();
    }

    int Flatland::dijkstraSearch(){
        map_.add("dijkstraSearch");
        ros::Duration duration(0.01);
        publish();

        Position startPosition (64,64);
        Position endPosition (-63,-63);

        ROS_INFO("Dijkstra");
        list<int> queue;
        bool visitedPath[128*128];
        double distPath[128*128];
        list<int> shortestPath;
        int count = 0;
        for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
        {
            visitedPath[count] = false;
            distPath[count] = DBL_MAX;
            // shortestPath[count] = -1;
            // queue.push_back(iterator.getLinearIndex());
            queue.push_back(count);
            count++;
        }

        distPath[0] = 0;
        bool endPos = false;
        int iterations = 0;
        double pathDistance = 0;
        while(!queue.empty() || endPos == false){
            int queueInt = getMinDistance(distPath,queue);
            // ROS_INFO("Queue Int %d", queueInt);
            // duration.sleep();
            if (queueInt == -1){
                endPos = true;
                break;
                // return iterations;
            }
            queue.remove(queueInt);

            Position queuePos;
            int indexY = (queueInt)/127;
            int indexX = (queueInt)%127;
            Index queueIndex(indexX,indexY);
            map_.getPosition(queueIndex,queuePos);

            int subXPos = queuePos.x() - 1;
            int subYPos = queuePos.y() - 1;
            for (int x = 0; x<3; x++){
                for (int y = 0; y<3; y++){
                    bool validPos = checkOpenPos(subXPos+x,subYPos+y,"dijkstraSearch");
                    if(validPos == true) {
                        int mapsubXPos = subXPos+x;
                        int mapsubYPos = subYPos+y;
                        Position subPos (mapsubXPos,mapsubYPos);

                        Index subIndex;
                        map_.getIndex(subPos,subIndex);
                        // int neighborInt = subIndex.getLinearIndex();
                        int neighborInt = getLinearIndexFromIndex(subIndex,map_.getSize());
                        bool found = (std::find(queue.begin(), queue.end(), neighborInt) != queue.end());

                        double neighborLength = getLength(queuePos,subPos);
                        double alt = distPath[queueInt] + neighborLength;
                        if (alt < distPath[neighborInt] && visitedPath[neighborInt] == false && found == true){
                            distPath[neighborInt] = alt;
                            visitedPath[neighborInt] = true;
                            ROS_INFO("visited path %d", neighborInt);
                            // if (pathDistance > distPath)
                            if (subPos == endPosition){
                                endPos = true;
                            }
                        }
                    }
                }
            }
            iterations++;
        }
        // int iterations = 0;
        grid_map::Matrix &data = map_["dijkstraSearch"];
        for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
        {
            const int i = iterator.getLinearIndex();
            if (visitedPath[i] == true){
                data(i) = 1.0;
                duration.sleep();
                publish();
                // iterations++;
            }
        }
        return iterations;
            
    }


    int Flatland::randomSearch(){
        map_.add("randomSearch");
        publish();
        Position startPosition (64,64);
        Position endPosition (-63,-63);
        std::srand(5324);
        std::rand();
        map_.atPosition("randomSearch",startPosition)= 1.0;
        Position currentPosition = startPosition;
        double currentXPos = currentPosition.x();
        double currentYPos = currentPosition.y();
        int xOffset = 0;
        int yOffset = 0;
        ros::Duration duration(0.001);
        bool endGoal = false;
        
        int iterations = 0;
        while (endGoal == false){
            bool posValid = false;
            int randCount = 0;
            while (posValid == false){
                xOffset = std::rand()%3-1;
                yOffset = std::rand()%3-1;
                ROS_INFO("Random: Moveoffset %d %d", xOffset, yOffset);
                randCount ++;
                posValid = checkOpenPos(currentXPos+xOffset, 
                                        currentYPos+yOffset,
                                        "randomSearch");
                if (randCount > 30 && posValid == false){
                    ROS_ERROR("No more available random moves");
                    posValid = true;
                    endGoal = true;
                }
            }
            Position newPos(currentXPos+xOffset, 
                            currentYPos+yOffset);
            ROS_INFO("Random: NewPos %f %f", newPos.x(), newPos.y());
            currentPosition = newPos;
            ROS_INFO("Random: Move to %f %f", currentPosition.x(), currentPosition.y());
            currentXPos = currentPosition.x();
            currentYPos = currentPosition.y();
            map_.atPosition("randomSearch",currentPosition)= 1.0;
            duration.sleep();
            publish();
            if (currentPosition == endPosition){
                endGoal = true;
            }
            iterations++;
            if (iterations > 20000){
                endGoal = true;
            }
        }
        return iterations;
        publish();
    }

    bool Flatland::checkOpenPos(int x, int y, std::string searchMap){
        grid_map::Matrix &data = map_["obstacle"];

        Position checkPos (x,y);
        // if (!map_.isInside(checkPos)){
        if ((x<-63)||(y<-63)||(x>64)||(y>64)){
            return false;
        }
        if (searchMap == "randomSearch"){
            searchMap = "obstacle";
        }
        double posValue = map_.atPosition("obstacle", checkPos);
        double alreadyVisit = map_.atPosition(searchMap, checkPos);
        if (( posValue == 1.0) || (alreadyVisit == 1.0)){
        // if (( posValue == 1.0) || ( posValue == 0.5) ){
            return false;
        }
            return true;
    }

    int Flatland::getMinDistance(double dist[], std::list<int> queue){
        
        double minDist = DBL_MAX;
        int minIndex = 0;
        for (int i = 0; i < 128*128; i++){
            if (dist[i] < minDist){
                bool found = (std::find(queue.begin(), queue.end(), i) != queue.end());
                if (found == true){
                    minDist = dist[i];
                    minIndex = i;
                }
            }
        }
        // ROS_INFO("MinDist %f", minDist);
        if (minDist == DBL_MAX){
            return -1;
        }
        return minIndex;
    }
    double Flatland::getLength(Position pos1, Position pos2){
        return sqrt(pow(pos1.x()-pos2.x(),2) + pow(pos1.y()-pos2.y(),2)*1.0);
    }


}