/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Just the card table
*/

#include <moveit_visual_tools/moveit_visual_tools.h> // simple tool for showing grasps

#ifndef BAXTER_PICK_PLACE__CUSTOM_ENVIRONMENT_
#define BAXTER_PICK_PLACE__CUSTOM_ENVIRONMENT_

namespace baxter_pick_place
{

// environment
static const std::string SUPPORT_SURFACE1_NAME = "monitor";
static const std::string SUPPORT_SURFACE2_NAME = "desk";
static const std::string SUPPORT_SURFACE3_NAME = "table";
static const std::string WALL1_NAME = "back_wall";
static const std::string WALL2_NAME = "right_wall";
static const std::string WALL3_NAME = "left_wall";

// table dimensions
static const double TABLE_HEIGHT = 0.715;

static const double TABLE_WIDTH  = 0.86;
static const double TABLE_DEPTH  = 0.86;
static const double TABLE_X = 0.4826 + TABLE_DEPTH / 2;
static const double TABLE_Y = 0; //-TABLE_WIDTH/2;

// block dimensions
static const double BLOCK_SIZE = 0.04;

void createEnvironment(moveit_visual_tools::MoveItVisualToolsPtr visual_tools_)
{
  visual_tools_->cleanupCO(SUPPORT_SURFACE1_NAME);
  visual_tools_->cleanupCO(SUPPORT_SURFACE2_NAME);
  visual_tools_->cleanupCO(WALL1_NAME);
  visual_tools_->cleanupCO(WALL2_NAME);
  visual_tools_->cleanupCO(WALL3_NAME);

  // --------------------------------------------------------------------------------------------
  // Add objects to scene

  // Tables                          x,         y,          angle, width,       height,       depth,       name
  visual_tools_->publishCollisionTable(TABLE_X,   TABLE_Y,    0,     TABLE_WIDTH, TABLE_HEIGHT, TABLE_DEPTH, SUPPORT_SURFACE3_NAME); // andy table
}

double getTableHeight(double floor_offset)
{
  return TABLE_HEIGHT + floor_offset + BLOCK_SIZE / 2;
}

void getTableWidthRange(double &y_min, double &y_max)
{
  y_min = TABLE_Y - TABLE_WIDTH / 2;
  y_max = TABLE_Y + TABLE_WIDTH / 2;
}

void getTableDepthRange(double &x_min, double &x_max)
{
  x_min = TABLE_X - TABLE_DEPTH / 2;
  x_max = TABLE_X + TABLE_DEPTH / 2;
}

} // namespace

#endif
