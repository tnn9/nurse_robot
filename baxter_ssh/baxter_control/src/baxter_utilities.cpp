/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, CU Boulder
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
 *   * Neither the name of CU Boulder nor the names of its
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

/**
 * \brief   Helper functions for controlling baxter
 * \author  Dave Coleman
 */

#include <baxter_control/baxter_utilities.h>

namespace baxter_control
{

BaxterUtilities::BaxterUtilities()
  : disabled_callback_called_(false),
    state_counter_(1)
{
  ROS_INFO_STREAM_NAMED("baxter_utilities","Loading Baxter utilities");
  ros::NodeHandle nh;

  // Preload Messages
  disable_msg_.data = false;
  enable_msg_.data = true;

  // ---------------------------------------------------------------------------------------------
  // Advertise services
  pub_baxter_enable_ = nh.advertise<std_msgs::Bool>("/robot/set_super_enable",10);
  pub_baxter_reset_ = nh.advertise<std_msgs::Empty>("/robot/set_super_reset",10);
  ROS_DEBUG_STREAM_NAMED("baxter_utilities","Publishing on topics /robot/set_super_enable and robot/set_super_reset");

  // ---------------------------------------------------------------------------------------------
  // Start the state subscriber
  sub_baxter_state_ = nh.subscribe<baxter_core_msgs::AssemblyState>(BAXTER_STATE_TOPIC,
                      1, &BaxterUtilities::stateCallback, this);

  // ---------------------------------------------------------------------------------------------
  // Shoulder enable disable buttons
  sub_shoulder_left_ = nh.subscribe<baxter_core_msgs::DigitalIOState>("/robot/digital_io/left_shoulder_button/state",
                       1, &BaxterUtilities::leftShoulderCallback, this);
  sub_shoulder_right_ = nh.subscribe<baxter_core_msgs::DigitalIOState>("/robot/digital_io/right_shoulder_button/state",
                        1, &BaxterUtilities::rightShoulderCallback, this);
  ROS_DEBUG_STREAM_NAMED("baxter_utilities","Subscribing to baxter_core_msgs");
}

void BaxterUtilities::setDisabledCallback(DisabledCallback callback)
{
  disabled_callback_ = callback;
}

bool BaxterUtilities::communicationActive()
{
  int count = 0;
  double expire_duration = 1.0;

  // Check that the message exists and the timestamp is no older than 1 second
  while( ros::ok() && (ros::Time::now() > baxter_state_timestamp_ + ros::Duration(expire_duration)))
  {
    if( count > 100 ) // 40 is an arbitrary number for when to assume no state is being published
    {
      if (baxter_state_timestamp_.toSec() == 0)
        ROS_WARN_STREAM_NAMED("utilities","No state message has been recieved on topic "
                               << BAXTER_STATE_TOPIC);
      else
        ROS_ERROR_STREAM_NAMED("utilities","Baxter state expired beyond timeout of " << expire_duration << " seconds. Diff: " 
                               << ros::Time::now() - baxter_state_timestamp_ << " sec");        
      return false;
    }

    ++count;
    //ROS_DEBUG_STREAM_NAMED("utilities","Waiting to not be expired " << ros::Time::now() - baxter_state_timestamp_);
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  return true;
}

bool BaxterUtilities::isEnabled(bool verbose)
{
  // Check communication
  if( !communicationActive() )
  {
    // Error message aready outputed
    return false;
  }

  // Check for estop
  if( baxter_state_->stopped == true )
  {
    // Skip the switch statments if we are not wanting verbose output
    if(!verbose)
      return false;

    std::string estop_button;
    switch( baxter_state_->estop_button )
    {
      case baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED:
        estop_button = "Robot is not stopped and button is not pressed";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_BUTTON_PRESSED:
        estop_button = "Pressed";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNKNOWN:
        estop_button = "STATE_UNKNOWN when estop was asserted by a non-user source";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_BUTTON_RELEASED:
        estop_button = "Was pressed, is now known to be released, but robot is still stopped.";
        break;
      default:
        estop_button = "Unkown button state code";
    }

    std::string estop_source;
    switch( baxter_state_->estop_source )
    {
      case baxter_core_msgs::AssemblyState::ESTOP_SOURCE_NONE:
        estop_source = "e-stop is not asserted";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_SOURCE_USER:
        estop_source = "e-stop source is user input (the red button)";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_SOURCE_UNKNOWN:
        estop_source = "e-stop source is unknown";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_SOURCE_FAULT:
        estop_source = "MotorController asserted e-stop in response to a joint fault";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_SOURCE_BRAIN:
        estop_source = "MotorController asserted e-stop in response to a lapse of the brain heartbeat";
        break;
      default:
        estop_source = "Unkown button source code";

    }

    ROS_ERROR_STREAM_NAMED("utilities","ESTOP Button State: '" << estop_button << "'. Source: '" << estop_source << "'");
    return false;
  }

  // Check for error
  if( baxter_state_->error == true )
  {
    if(verbose)
      ROS_ERROR_STREAM_NAMED("utilities","Baxter has an error :(  State: \n" << *baxter_state_ );
    return false;
  }

  // Check enabled
  if( baxter_state_->enabled == false )
  {
    if(verbose)
      ROS_ERROR_STREAM_NAMED("utilities","Baxter is not enabled.  State: \n" << *baxter_state_ );

    return false;
  }

  return true;
}

void BaxterUtilities::stateCallback(const baxter_core_msgs::AssemblyStateConstPtr& msg)
{
  baxter_state_ = msg;
  baxter_state_timestamp_ = ros::Time::now();

  // Check for errors every CHECK_FREQ refreshes to save computation
  static const int CHECK_FREQ = 25; //50;
  if( state_counter_ % CHECK_FREQ == 0 )
  {
    if( !isEnabled() ) // baxter is disabled
    {
      if (disabled_callback_called_ == false)
      {
        // Call the parent classes' callback if they've provided one
        if (disabled_callback_)
          disabled_callback_();

        disabled_callback_called_ = true;
      }
    }
    else // baxter is not disabled
    {
      disabled_callback_called_ = false;
    }

    // Reset the counter so it doesn't overflow
    state_counter_ = 0;
  }

  state_counter_++;
}

void BaxterUtilities::leftShoulderCallback(const baxter_core_msgs::DigitalIOStateConstPtr& msg)
{
  if (msg->state == 0)
  {
    enableBaxter();
  }
}

void BaxterUtilities::rightShoulderCallback(const baxter_core_msgs::DigitalIOStateConstPtr& msg)
{
  if (msg->state == 0)
  {
    disableBaxter();
  }
}

bool BaxterUtilities::enableBaxter()
{
  ROS_INFO_STREAM_NAMED("utility","Enabling Baxter");

  // Check if we need to do anything
  if( isEnabled(false) )
    return true;

  // Wait for state msg to be recieved
  if( !communicationActive() )
    return false;
  
  // Reset Baxter
  if( !resetBaxter() )
    return false;

  // Attempt to enable baxter
  pub_baxter_enable_.publish(enable_msg_);
  ros::Duration(0.5).sleep();

  // Check if enabled
  int count = 0;
  while( ros::ok() && !isEnabled(true) )
  {
    if( count > 20 ) // 20 is an arbitrary number for when to assume its not going to enable
    {
      ROS_ERROR_STREAM_NAMED("utilities","Giving up on waiting");
      return false;
    }

    ++count;
    ros::Duration(0.05).sleep();
  }

  return true;
}

bool BaxterUtilities::disableBaxter()
{
  ROS_INFO_STREAM_NAMED("utility","Disabling Baxter");

  // Wait for state msg to be recieved
  if( !communicationActive() )
    return false;

  pub_baxter_enable_.publish(disable_msg_);
  ros::Duration(0.5).sleep();

  // Check it enabled
  int count = 0;
  while( ros::ok() && baxter_state_->enabled == true )
  {
    if( count > 20 ) // 20 is an arbitrary number for when to assume its not going to enable
    {
      ROS_ERROR_STREAM_NAMED("utilities","Failed to disable Baxter");
      return false;
    }

    ++count;
    ros::Duration(0.05).sleep();
  }

  return true;
}

bool BaxterUtilities::resetBaxter()
{
  ROS_INFO_STREAM_NAMED("utility","Resetting Baxter");

  // Wait for state msg to be recieved
  if( !communicationActive() )
    return false;

  // Attempt to reset and enable robot
  pub_baxter_reset_.publish(empty_msg_);
  ros::Duration(0.5).sleep();

  return true;
}



} //namespace

