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
 *  @file    FrontierExplorer.cpp
 *  @author  Harsh Kakashaniya and Rohitkrishna Nambiar
 *  @date    12/04/2018
 *  @version 1.0
 *  @copyright BSD 3-Clause
 *
 *  @brief FrontierExplorer class
 *
 *  @section DESCRIPTION
 *
 *  FrontierExplorer class implementation
 */

#include "FrontierExplorer.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <utility>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

FrontierExplorer::FrontierExplorer() {
  // Initialize publisher topic
  velPub = nh.advertise < geometry_msgs::Twist
      > ("/mobile_base/commands/velocity", 100);

  // Sunscriber to map message
  mapSub = nh.subscribe("/map", 100, &FrontierExplorer::processOccupancyGrid,
                        this);

  // Set start velocity message to zero
  velMsg.linear.x = 0.0;
  velMsg.linear.y = 0.0;
  velMsg.linear.z = 0.0;
  velMsg.angular.x = 0.0;
  velMsg.angular.y = 0.0;
  velMsg.angular.z = 0.0;

  // Publish the velocity
  velPub.publish(velMsg);

  // Visualization markers for frontier and centroids
  // Frontiers segmented based on clusters
  frontierMarkerPub = nh.advertise < visualization_msgs::MarkerArray
      > ("/frontier_marker_array", 1);

  // Publish all frontiers
  allFrontierPub = nh.advertise < visualization_msgs::MarkerArray
      > ("/all_frontier_marker_array", 1);

  // Only clusters
  frontierClusterPub = nh.advertise < visualization_msgs::MarkerArray
      > ("/frontier_clustor_array", 1);

  reachAvoidPub = nh.advertise < visualization_msgs::MarkerArray
      > ("/reach_avoid_region", 1);

  ROS_INFO("New frontier exploration turtle bot created.");
}

FrontierExplorer::~FrontierExplorer() {
  // Set velocity to zero on exit
  velMsg.linear.x = 0.0;
  velMsg.linear.y = 0.0;
  velMsg.linear.z = 0.0;
  velMsg.angular.x = 0.0;
  velMsg.angular.y = 0.0;
  velMsg.angular.z = 0.0;

  // Publish the velocity
  velPub.publish(velMsg);
}

void FrontierExplorer::rotate360() {
  velMsg.linear.x = 0.0;
  velMsg.linear.y = 0.0;
  velMsg.linear.z = 0.0;
  velMsg.angular.x = 0.0;
  velMsg.angular.y = 0.0;
  velMsg.angular.z = 0.5;
  ros::Time begin = ros::Time::now();
  ros::Duration waitTime = ros::Duration(14);
  ros::Time end = begin + waitTime;
  std::cout << "[ Start time:]" << begin << std::endl;
  while (ros::Time::now() < end && ros::ok()) {
    // Publish the velocity
    velPub.publish(velMsg);
  }
  ROS_INFO_STREAM("[ End time:]" << ros::Time::now());
}

void FrontierExplorer::processOccupancyGrid(const nav_msgs::OccupancyGrid
                                            ::ConstPtr& gridMsg) {
  std::cout << "\n\n";
  ROS_INFO("Occupancy message post-process called..");
  int currwidth = gridMsg->info.width;  // x
  int currheight = gridMsg->info.height;  // y
  double currreso = gridMsg->info.resolution;
  geometry_msgs::Point currcenter = gridMsg->info.origin.position;
  ROS_INFO_STREAM("[MAP INFO] Width: " << currwidth << ", Height: "
          << currheight
          << ", Resolution: " << currreso << ", Origin: "
          << gridMsg->info.origin.position.x << ","
          << gridMsg->info.origin.position.y);

  slamMap.updateMap(currwidth, currheight, currreso, currcenter, gridMsg);
}

int FrontierExplorer::getNearestCluster(
    std::vector<std::pair<double, double>> centers) {
  tf::StampedTransform transform;
  turtleFrameListener.lookupTransform("/map", "/base_link", ros::Time(0),
                                      transform);

  double turtleX = transform.getOrigin().x();
  double turtleY = transform.getOrigin().y();

  ROS_INFO_STREAM("Current turtle location x:" << turtleX << ", y:" << turtleY);

  // Index and distance to store the closest frontier
  int closestFrontierIndex = -1;
  double distance = -1;

  // Loop through all the clusters
  int loopIndex = 0;
  for (auto center : centers) {
    double currDistance = std::hypot(center.first - turtleX,
                                     center.second - turtleY);
    if ((distance == -1 || (currDistance < distance && currDistance > 1.5))
        && checkReachAvoid(center)) {
      distance = currDistance;
      closestFrontierIndex = loopIndex;
    }
    loopIndex++;
  }

  ROS_INFO_STREAM("Found closest cluster at number: " << closestFrontierIndex);

  return closestFrontierIndex;
}

bool FrontierExplorer::checkReachAvoid(std::pair<double, double> goalPoint) {
  double x = goalPoint.first;
  double y = goalPoint.second;

  for (auto row : reachAvoid) {
    double currDistance = std::hypot(row.first - x, row.second - y);
    if (currDistance < 1) {
      ROS_INFO_STREAM(
          "Found a point close by that should not be visited. "
          "Skipping this point..");
      // False means we can not go to that point as it has been added in
      // reach-avoid list
      return false;
    }
  }
  // True means we can go to that point as it has not been added in reach-avoid
  return true;
}

void FrontierExplorer::moveTurtle(
    std::vector<std::pair<double, double>> centers, int id) {
  double goalPointX = centers[id].first;
  double goalPointY = centers[id].second;

  // Create move base goal with params
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = goalPointX;
  goal.target_pose.pose.position.y = goalPointY;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO_STREAM(
      "Navigating to point x:" << goalPointX << ", y:" << goalPointY
          << " on the map");

  actionlib::SimpleActionClient < move_base_msgs::MoveBaseAction
      > ac("move_base", true);

  // wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO_STREAM("Waiting for the move_base action server to come up");
  }
  ROS_INFO_STREAM("Move base action server online..");
  ac.sendGoal(goal);
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO_STREAM("Cluster centroid reached.");
  else
    ROS_INFO_STREAM(
        "Could not reach cluster centroid. Adding it into"
        " regions not to visit again..");
  reachAvoid.push_back(centers[id]);
}

void FrontierExplorer::visualizeClusterCenters(std::vector<std::pair<double,
                                                double>> centers, int id) {
  visualization_msgs::MarkerArray clusterMarkerArray;
  int clusterIndex = 0;
  for (auto center : centers) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "cluster_center";
    marker.id = clusterIndex;
    marker.type = visualization_msgs::Marker::CUBE;

    marker.action = visualization_msgs::Marker::ADD;

    if (clusterIndex == id) {
      // Define the scale (meter scale)
      marker.scale.x = 0.08;
      marker.scale.y = 0.08;
      marker.scale.z = 0.08;

      // Set the color
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.6;
    } else {
      // Define the scale (meter scale)
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;

      // Set the color
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.8;
    }
    marker.lifetime = ros::Duration();

    marker.pose.position.x = center.first;
    marker.pose.position.y = center.second;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1.0;

    clusterIndex++;
    clusterMarkerArray.markers.push_back(marker);
  }

  frontierClusterPub.publish(clusterMarkerArray);
  ROS_INFO("Total %d markers published.", clusterIndex);
}

void FrontierExplorer::visualizeClusterFrontiers() {
  auto frontierCluster = slamMap.getFrontierCluster();
  std::vector<std::vector<MapNode>> map = slamMap.getMap();
  // 8 different colors(r,g,b). Loop after we get there
//  int colorSize = 8;
  std::vector<std::tuple<double, double, double>> colors;
  colors.push_back(std::make_tuple(0.5, 0.0, 0.0));
  colors.push_back(std::make_tuple(0.0, 0.0, 1.0));
  colors.push_back(std::make_tuple(0.0, 1.0, 0.0));
  colors.push_back(std::make_tuple(0.0, 1.0, 1.0));
  colors.push_back(std::make_tuple(1.0, 0.0, 0.0));
  colors.push_back(std::make_tuple(1.0, 0.0, 1.0));
  colors.push_back(std::make_tuple(1.0, 1.0, 0.0));
  colors.push_back(std::make_tuple(1.0, 1.0, 1.0));

  int colorCounter = 1;
  int frontierIndex = 0;
  int frontierNodeIndex = 0;
  // Loop through the clusters
  for (auto row : frontierCluster) {
    visualization_msgs::MarkerArray clusterCenterMarkerArray;
    for (auto point : row) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time();
      marker.ns = "cluster_marker_frontier";
      marker.id = frontierNodeIndex;
      marker.type = visualization_msgs::Marker::CUBE;

      marker.action = visualization_msgs::Marker::ADD;

      // Define the scale (meter scale)
      marker.scale.x = 0.02;
      marker.scale.y = 0.02;
      marker.scale.z = 0.02;

      // Set the color
      marker.color.r = std::get < 0 > (colors[colorCounter - 1]);
      marker.color.g = std::get < 1 > (colors[colorCounter - 1]);
      marker.color.b = std::get < 2 > (colors[colorCounter - 1]);
      //      marker.color.r = 1.0;
      //      marker.color.g = 0.0;
      //      marker.color.b = 0.0;
      marker.color.a = 0.8;
      marker.lifetime = ros::Duration();

      marker.pose.position.x = map[point.first][point.second].getX();
      marker.pose.position.y = map[point.first][point.second].getY();
      marker.pose.position.z = 0;
      marker.pose.orientation.w = 1.0;

      clusterCenterMarkerArray.markers.push_back(marker);
      frontierNodeIndex++;
    }
    frontierIndex++;
    // Adjust for color
    if (colorCounter % 8 == 0) {
      colorCounter = 1;
    } else {
      colorCounter++;
    }
    frontierMarkerPub.publish(clusterCenterMarkerArray);
    ROS_INFO_STREAM(
        "Published cluster no: " << frontierIndex << ", with color index: "
            << colorCounter);
  }
}

void FrontierExplorer::publishFrontierPoints(int count) {
  //  ROS_INFO("Publishing frontier markers... ");
  int width = slamMap.getmapWidth();
  int height = slamMap.getmapHeight();

  std::vector<std::vector<MapNode>> map = slamMap.getMap();

  visualization_msgs::MarkerArray markerArray;

  int markerCount = 0;
  int markerLimit = count;

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      if (map[i][j].getisFrontier()) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time();
        marker.ns = "Frontier";
        marker.id = markerCount;
        marker.type = visualization_msgs::Marker::SPHERE;

        marker.action = visualization_msgs::Marker::ADD;

        // Define the scale (meter scale)
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;

        // Set the color
        marker.color.r = 0.0f;
        marker.color.g = 0.41f;
        marker.color.b = 0.70f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();

        marker.pose.position.x = map[i][j].getX();
        marker.pose.position.y = map[i][j].getY();
        marker.pose.position.z = 0;
        marker.pose.orientation.w = 1.0;

        markerCount++;
        if (markerCount < markerLimit) {
          markerArray.markers.push_back(marker);
        }
      }
    }
  }

  allFrontierPub.publish(markerArray);
  ROS_INFO("Total %d markers published.", markerCount);
}

void FrontierExplorer::visualizeReachAvoid() {
  visualization_msgs::MarkerArray clusterMarkerArray;
  int clusterIndex = 0;
  for (auto center : reachAvoid) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "reach_avoid";
    marker.id = clusterIndex;
    marker.type = visualization_msgs::Marker::CUBE;

    marker.action = visualization_msgs::Marker::ADD;

    // Define the scale (meter scale)
    marker.scale.x = 1.00;
    marker.scale.y = 1.00;
    marker.scale.z = 0.01;

    // Set the color
    marker.color.r = 1.0f;
    marker.color.g = 0.5f;
    marker.color.b = 0.5f;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration();

    marker.pose.position.x = center.first;
    marker.pose.position.y = center.second;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1.0;

    clusterIndex++;
    clusterMarkerArray.markers.push_back(marker);
  }

  reachAvoidPub.publish(clusterMarkerArray);
  ROS_INFO("Total %d markers published in reach avoid.", clusterIndex);
}

void FrontierExplorer::explore() {
  // Set loop frequency
  ros::Rate loop_rate(1);
  bool shouldRotate = true;
  loop_rate.sleep();
  ros::spinOnce();
  ROS_INFO_STREAM("Starting prometheus frontier exploration in 5 seconds...");
  ros::Duration(5).sleep();
  while (ros::ok()) {
//    ROS_INFO_STREAM("\n\n\n");
    ROS_INFO_STREAM("#################################");

    if (shouldRotate) {
      rotate360();
      shouldRotate = false;
      ROS_INFO("Finished rotating turtlebot");
    }

    // Check frontiers
    int count = slamMap.getFrontiers();

    slamMap.getClusters(20);

    // Get cluster centroids
    std::vector<std::pair<double, double>> clusterCenters =
        slamMap
        .getClusterCentroids();

    // Get nearest cluster index
    int id = getNearestCluster(clusterCenters);

    if (id == -1) {
      ROS_INFO_STREAM("Could not find any more frontiers..");
      break;
    }

    // Visualize cluster centroids
    visualizeClusterCenters(clusterCenters, id);

    // Visualize cluster frontiers
    visualizeClusterFrontiers();

    // Visualize all frontiers
    publishFrontierPoints(count);

    // Visualize reach avoid
    visualizeReachAvoid();

    // Move to the cluster center
    moveTurtle(clusterCenters, id);

    // Spin once to check for callbacks
    ros::spinOnce();

    // Sleep for desired frequency
    loop_rate.sleep();
  }

  ROS_INFO_STREAM("Finished exploring the map. Exiting..");
}
