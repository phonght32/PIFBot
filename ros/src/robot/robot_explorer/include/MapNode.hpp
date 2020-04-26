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
 *  @file    MapNode.hpp
 *  @author  Harsh Kakashaniya and Rohitkrishna Nambiar
 *  @date    12/04/2018
 *  @version 1.0
 *  @copyright BSD 3-Clause
 *
 *  @brief MapNode class header file
 *
 *  @section DESCRIPTION
 *
 *  MapNode class header declaration
 */

  #ifndef INCLUDE_MAPNODE_HPP_
  #define INCLUDE_MAPNODE_HPP_

#include <cstdint>

/**
 * @brief MapNode Class
 *
 * Class for storing occupancy grid map nodes
 */
class MapNode {
 public:
  /**
   *  @brief Default constructor for MapNode class
   *
   *  @param none
   *
   *  @return none
   */
  MapNode();

  /**
   *  @brief Destructor for MapNode class
   *
   *  @param none
   *
   *  @return none
   */
  ~MapNode();

  /**
   *  @brief Function to get x-coordinate for map node
   *
   *  @param none
   *
   *  @return float x-coordinate of node
   */
  float getX();

  /**
   *  @brief Function to get y-coordinate for map node
   *
   *  @param none
   *
   *  @return float y-coordinate of node
   */
  float getY();

  /**
   *  @brief Function to get probability for map node
   *
   *  @param none
   *
   *  @return int8_t probability of node
   */
  int8_t getProbability();

  /**
   *  @brief Function to set x-coordinate for map node
   *
   *  @param float x-coordinate of node
   *
   *  @return void
   */
  void setX(float value);

  /**
   *  @brief Function to set y-coordinate for map node
   *
   *  @param float y-coordinate of node
   *
   *  @return void
   */
  void setY(float value);

  /**
   *  @brief Function to set probability for map node
   *
   *  @param int8_t probability of node
   *
   *  @return voide
   */
  void setProbability(int8_t value);

  /**
   *  @brief Function to get isfrontier flag for map node
   *
   *  @param none
   *
   *  @return bool isfrontier flag
   */
  bool getisFrontier();

  /**
   *  @brief Function to set isfrontier flag for map node
   *
   *  @param bool isfrontier flag
   *
   *  @return void
   */
  void setisFrontier(bool value);

  /**
   *  @brief Function to get frontier index flag for map node
   *
   *  @param none
   *
   *  @return int frontier index
   */
  int getFrontierIndex();

  /**
   *  @brief Function to set frontier index flag for map node
   *
   *  @param int frontier index
   *
   *  @return void
   */
  void setFrontierIndex(int value);

 private:
  // Node x-coordinate
  float x;

  // Node x-coordinate
  float y;

  // Node probability
  int8_t probability;

  // Flag to check if node is frontier
  bool isFrontier;

  // Node frontier index
  int frontierIndex;
};

#endif  // INCLUDE_MAPNODE_HPP_
