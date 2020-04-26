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
 *  @file    MapNode.cpp
 *  @author  Harsh Kakashaniya and Rohitkrishna Nambiar
 *  @date    12/04/2018
 *  @version 1.0
 *  @copyright BSD 3-Clause
 *
 *  @brief MapNode class implementation file
 *
 *  @section DESCRIPTION
 *
 *  Mapnode class implementation file for prometheus_frontier_exploration package.
 */
#include "MapNode.hpp"
#include <cstdint>

MapNode::MapNode() {
  // Set default values for nodes
  isFrontier = false;
  y = -1;
  x = -1;
  probability = -1;
  frontierIndex = -1;
}

MapNode::~MapNode() {
}

float MapNode::getX() {
  return x;
}

float MapNode::getY() {
  return y;
}

int8_t MapNode::getProbability() {
  return probability;
}

void MapNode::setX(float value) {
  x = value;
}

void MapNode::setY(float value) {
  y = value;
}

void MapNode::setProbability(int8_t value) {
  probability = value;
}

bool MapNode::getisFrontier() {
  return isFrontier;
}

void MapNode::setisFrontier(bool value) {
  isFrontier = value;
}

int MapNode::getFrontierIndex() {
  return frontierIndex;
}

void MapNode::setFrontierIndex(int value) {
  frontierIndex = value;
}
