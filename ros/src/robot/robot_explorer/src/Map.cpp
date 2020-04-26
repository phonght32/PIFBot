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
 *  @file    Map.cpp
 *  @author  Harsh Kakashaniya and Rohitkrishna Nambiar
 *  @date    12/04/2018
 *  @version 1.0
 *  @copyright BSD 3-Clause
 *
 *  @brief Map class implementation file
 *
 *  @section DESCRIPTION
 *
 *  Map class implementation
 */
#include "Map.hpp"
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <vector>
#include <cstdint>
#include <string>
#include <utility>

Map::Map() {
  // Set Map parameters on object creation
  mapSet = false;
  mapHeight = 0;
  mapWidth = 0;
  mapReso = 0.05;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;
}

Map::~Map() {
}

void Map::setMapSet(bool value) {
  mapSet = value;
}

bool Map::getMapSet() {
  return mapSet;
}

int Map::getmapHeight() {
  return mapHeight;
}

int Map::getmapWidth() {
  return mapWidth;
}

double Map::getmapReso() {
  return mapReso;
}

void Map::setmapHeight(int value) {
  mapHeight = value;
}

void Map::setmapWidth(int value) {
  mapWidth = value;
}

void Map::setmapReso(double value) {
  mapReso = value;
}

void Map::setOrigin(geometry_msgs::Point point) {
  origin = point;
}

geometry_msgs::Point Map::getOrigin() {
  return origin;
}

const std::vector<std::vector<MapNode>>& Map::getMap() {
  return map;
}

bool Map::updateMapParams(int currentWidth, int currentHeight,
                          double currentReso,
                          geometry_msgs::Point currCenter) {
  bool updateFlag = false;
  if (currentWidth != mapWidth) {
    ROS_INFO_STREAM(
        "Map width updated from " << mapWidth << " to " << currentWidth);
    mapWidth = currentWidth;
    updateFlag = true;
  }

  if (currentHeight != mapHeight) {
    ROS_INFO_STREAM(
        "Map height updated from " << mapHeight << " to " << currentHeight);
    mapHeight = currentHeight;
    updateFlag = true;
  }

  if (currentReso != mapReso) {
    ROS_INFO_STREAM(
        "Map resolution updated from " << mapReso << " to " << currentReso);
    mapReso = currentReso;
    updateFlag = true;
  }

  if (currCenter.x != origin.x || currCenter.y != origin.y) {
    origin.x = currCenter.x;
    origin.y = currCenter.y;
    origin.z = currCenter.z;
    ROS_INFO_STREAM(
        "Map origin updated at x:" << origin.x << ", y:" << origin.y);
  }
  return updateFlag;
}

void Map::updateMap(int currentWidth, int currentHeight, double currentReso,
                    geometry_msgs::Point mapCenter,
                    const nav_msgs::OccupancyGrid::ConstPtr& gridMsg) {
  //  ROS_INFO("Update Map function received with width:%d, height:%d",
  //           currentWidth, currentHeight);
  /* Check is map has been updated. If yes, set mapSet flag to false to reset
   the map */
  if (updateMapParams(currentWidth, currentHeight, currentReso, mapCenter)) {
    mapSet = false;
    ROS_INFO(
        "Map reset as one of the parameter has been updated."
"Will initialize again..");
  }

  int mapIter = 0;
  if (!mapSet) {
    mapSet = true;
    // Clearing out the vector incase re-initialized
    map.clear();

    // Loop to fill the map. We go from every width for each height
    for (int i = 0; i < currentHeight; i++) {
      std::vector<MapNode> rowNodes;
      for (int j = 0; j < currentWidth; j++) {
        MapNode currentNode;

        // Calculate x and y
        float x = j * mapReso + origin.x;
        float y = i * mapReso + origin.y;

        // Update in each node of map
        currentNode.setX(x);
        currentNode.setY(y);
        currentNode.setProbability(gridMsg->data[mapIter]);
        rowNodes.push_back(currentNode);
        mapIter++;
      }
      map.push_back(rowNodes);
    }
    ROS_INFO_STREAM(
        "Map Initialized. Current w:" << map[0].size() << ", h:" << map.size()
            << ", resolution: " << mapReso << ", origin x: " << origin.x
            << ", y: " << origin.y);
  } else {
    for (int i = 0; i < currentHeight; i++) {
      for (int j = 0; j < currentWidth; j++) {
        map[i][j].setProbability(gridMsg->data[mapIter]);
        mapIter++;
      }
    }
    ROS_INFO_STREAM(
        "Map Updated. Current w:" << map[0].size() << " ,h:" << map.size()
            << ", resolution: " << mapReso << ", origin x: " << origin.x
            << ", y: " << origin.y);
  }
}

int Map::getFrontiers() {
  int frontierCount = 0;
//  ROS_INFO_STREAM("Getting frontiers..");
  ROS_INFO("Getting frontiers for map with w:%d, h:%d", mapWidth, mapHeight);
    for (int i = 0; i < mapHeight; i++) {
      for (int j = 0; j < mapWidth; j++) {
        if (map[i][j].getProbability() == 0) {
          bool frontierFlag = false;
          // Check if neighbor is unexplored node
          for (int k = i - 1; k <= i + 1; k++) {
            for (int l = j - 1; l <= j + 1; l++) {
            if (k >= 0 && l >= 0 && k < mapHeight && l < mapWidth
                  && map[k][l].getProbability() == -1) {
                // Set flag to true
                frontierFlag = true;
              }
            }
          }
          // Update in the map
          map[i][j].setisFrontier(frontierFlag);
          if (frontierFlag) {
            frontierCount++;
            }
          }
        // We also set its frontier id to -1
        map[i][j].setFrontierIndex(-1);
        }
      }

  ROS_INFO("Updated frontier flag and index with frontier count:%d",
           frontierCount);
  return frontierCount;
}

int Map::getClusters(int threshold) {
  // Declare structure to store clusters
  std::vector < std::vector<std::pair<int, int>> > clusters;

  // Left half algorithm
  for (int i = 0; i < mapHeight; i++) {
    for (int j = 0; j < mapWidth; j++) {
      if (!map[i][j].getisFrontier()) {
        continue;
      } else if (i - 1 >= 0 && j - 1 >= 0
          && map[i - 1][j - 1].getFrontierIndex() != -1) {
        map[i][j].setFrontierIndex(map[i - 1][j - 1].getFrontierIndex());
        clusters[map[i][j].getFrontierIndex()].push_back(std::make_pair(i, j));
      } else if (i - 1 >= 0 && map[i - 1][j].getFrontierIndex() != -1) {
        map[i][j].setFrontierIndex(map[i - 1][j].getFrontierIndex());
        clusters[map[i][j].getFrontierIndex()].push_back(std::make_pair(i, j));
      } else if (j - 1 >= 0 && map[i][j - 1].getFrontierIndex() != -1) {
        map[i][j].setFrontierIndex(map[i][j - 1].getFrontierIndex());
        clusters[map[i][j].getFrontierIndex()].push_back(std::make_pair(i, j));
      } else if (i - 1 >= 0 && j < mapWidth - 1
          && map[i - 1][j + 1].getFrontierIndex() != -1) {
        map[i][j].setFrontierIndex(map[i - 1][j + 1].getFrontierIndex());
        clusters[map[i][j].getFrontierIndex()].push_back(std::make_pair(i, j));
      } else if ((i - 1 >= 0 && j - 1 >= 0
          && map[i - 1][j].getFrontierIndex() == -1
          && map[i][j - 1].getFrontierIndex() == -1)
          || (i - 1 < 0 && j - 1 >= 0 && map[i][j - 1].getFrontierIndex() == -1)
          || (i - 1 >= 0 && j - 1 < 0 &&
          map[i - 1][j].getFrontierIndex() == -1)) {
        map[i][j].setFrontierIndex(clusters.size());
        std::vector<std::pair<int, int>> coordinates;
        coordinates.push_back(std::make_pair(i, j));
        clusters.push_back(coordinates);
      }
    }
  }

  // Right bruno algorithm

//  for (int i = 0; i < mapHeight; i++) {
//    for (int j = 0; j < mapWidth; j++) {
//      if (!map[i][j].getisFrontier()) {
//        continue;
//      } else if (i - 1 >= 0 && j < mapWidth - 1
//          && map[i - 1][j + 1].getFrontierIndex() != -1) {
//        map[i][j].setFrontierIndex(map[i - 1][j + 1].getFrontierIndex());
//        clusters[map[i][j].getFrontierIndex()]
//        .push_back(std::make_pair(i, j));
//      }
//    }
//  }

  // Filtering out smaller clusters
  frontierCluster.clear();
  for (auto row : clusters) {
    if (row.size() > threshold) {
      frontierCluster.push_back(row);
    }
  }

  ROS_INFO_STREAM("Number of clusters: " << frontierCluster.size());
  return frontierCluster.size();
}

std::vector<std::pair<double, double>> Map::getClusterCentroids() {
  std::vector<std::pair<double, double>> centroids;
  for (auto row : frontierCluster) {
    double sumX = 0, sumY = 0;
    for (auto point : row) {
      int i = 0, j = 0;
      i = point.first;
      j = point.second;
      sumX = sumX + map[i][j].getX();
      sumY = sumY + map[i][j].getY();
    }
    sumX = sumX / row.size();
    sumY = sumY / row.size();
    ROS_INFO_STREAM("Centroid x: " << sumX << ", y: " << sumY);
    centroids.push_back(std::make_pair(sumX, sumY));
  }
  return centroids;
}

const std::vector<std::vector<std::pair<int, int>>>& Map::getFrontierCluster() {
  return frontierCluster;
}
