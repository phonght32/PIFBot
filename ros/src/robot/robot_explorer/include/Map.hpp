/*
 BSD 3-Clause License

 Copyright (c) 2018, Rohitkrishna Nambiar,  Harsh Kakashaniya
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
 contributors may be used to endorse or promote products derived from
 this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  @file    Map.hpp
 *  @author  Harsh Kakashaniya and Rohitkrishna Nambiar
 *  @date    12/04/2018
 *  @version 1.0
 *  @copyright BSD 3-Clause
 *
 *  @brief Map class header file
 *
 *  @section DESCRIPTION
 *
 *  Map class header declaration
 */

  #ifndef INCLUDE_MAP_HPP_
  #define INCLUDE_MAP_HPP_

#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <vector>
#include <cstdint>
#include <string>
#include <utility>
#include "ros/ros.h"
#include "MapNode.hpp"

/**
 * @brief Map Class
 *
 * Class for storing occupancy grid map
 */
class Map {
 public:
  /**
   *  @brief Default constructor for Map class
   *
   *  @param none
   *
   *  @return none
   */
  Map();

  /**
   *  @brief Destructor for Map class
   *
   *  @param none
   *
   *  @return none
   */
  ~Map();

  /**
   *  @brief Function to set flag for map initialized
   *
   *  @param bool flag value to indicate initialized or not
   *
   *  @return void
   */
  void setMapSet(bool value);

  /**
   *  @brief Function to get flag for map initialized
   *
   *  @param none
   *
   *  @return bool flag value to indicate initialized or not
   */
  bool getMapSet();

  /**
   *  @brief Function to get map height
   *
   *  @param none
   *
   *  @return int map height
   */
  int getmapHeight();

  /**
   *  @brief Function to get map width
   *
   *  @param none
   *
   *  @return int map width
   */
  int getmapWidth();

  /**
   *  @brief Function to get map resolution
   *
   *  @param none
   *
   *  @return double map resolution
   */
  double getmapReso();

  /**
   *  @brief Function to set map height
   *
   *  @param int map height
   *
   *  @return void
   */
  void setmapHeight(int value);

  /**
   *  @brief Function to set map width
   *
   *  @param int map width
   *
   *  @return void
   */
  void setmapWidth(int value);

  /**
   *  @brief Function to set map resolution
   *
   *  @param double map resolution
   *
   *  @return void
   */
  void setmapReso(double value);

  /**
   *  @brief Function to set map origin
   *
   *  @param geometry_msgs::Point map origin
   *
   *  @return void
   */
  void setOrigin(geometry_msgs::Point point);

  /**
   *  @brief Function to get map origin
   *
   *  @param none
   *
   *  @return geometry_msgs::Point map origin
   */
  geometry_msgs::Point getOrigin();

  /**
   *  @brief Function to get map
   *
   *  @param none
   *
   *  @return std::vector<std::vector<MapNode>>& map
   */
  const std::vector<std::vector<MapNode>>& getMap();

  /**
   *  @brief Function to update map parameters
   *
   *  @param int map width
   *  @param int map height
   *  @param double map resolution
   *  @param geometry_msgs::Point map center/origin
   *
   *  @return bool update status
   */
  bool updateMapParams(int currentWidth, int currentHeight, double currentReso,
                       geometry_msgs::Point mapCenter);

  /**
   *  @brief Function to update map
   *
   *  @param int map width
   *  @param int map height
   *  @param double map resolution
   *  @param geometry_msgs::Point map center/origin
   *  @param  nav_msgs::OccupancyGrid::ConstPtr& occupancy grid map message
   *
   *  @return bool update status
   */
  void updateMap(int currentWidth, int currentHeight, double currentReso,
                 geometry_msgs::Point mapCenter,
                 const nav_msgs::OccupancyGrid::ConstPtr& msg);
  /**
   *  @brief Function to get frontiers from occupancy grid message
   *
   *  @param none
   *
   *  @return int count of frontiers
   */
  int getFrontiers();

  /**
   *  @brief Function to make clusters from array of frontier location
   *
   *  @param int threshold of frontiers to form cluster
   *
   *  @return int number of clusters
   */
  int getClusters(int threshold);

  /**
   *  @brief Function to calculate cluster centroids
   *
   *  @param none
   *
   *  @return std::vector<std::pair<double, double>> cluster centroid locations
   */
  std::vector<std::pair<double, double>> getClusterCentroids();

  /**
   *  @brief Function to get cluster frontiers
   *
   *  @param none
   *
   *  @return std::vector<std::vector<std::pair<int, int>>> cluster frontiers
   */
  const std::vector<std::vector<std::pair<int, int>>>& getFrontierCluster();

 private:
  // Map initialized flag
  bool mapSet;

  // Map height, width and resolution
  int mapHeight;
  int mapWidth;
  float mapReso;

  // Occupancy grid
  std::vector<std::vector<MapNode>> map;

  // Map origin
  geometry_msgs::Point origin;

  // Structure to hold frontier clusters
  std::vector<std::vector<std::pair<int, int>>> frontierCluster;
};

#endif  // INCLUDE_MAP_HPP_
