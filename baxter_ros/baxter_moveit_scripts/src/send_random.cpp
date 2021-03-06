#include <ros/ros.h>

// Baxter Utilities
#include <baxter_control/baxter_utilities.h>
#include <baxter_moveit_scripts/baxter_move_group_interface.h>
#include <moveit_simple_grasps/grasp_data.h>

// Custom
//#include <baxter_pick_place/custom_environment2.h>

// Visualization
#include <moveit_visual_tools/moveit_visual_tools.h> // simple tool for showing graspsp

#include <std_msgs/Bool.h>
#include <baxter_core_msgs/HeadPanCommand.h>


namespace baxter_pick_place
{

//static const std::string PLANNING_GROUP = "right_arm";
static const std::string PLANNING_GROUP = "both_arms";

class SendRandom
{
public:

  // class for publishing stuff to rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // baxter helper
  baxter_control::BaxterUtilities baxter_util_;
  baxter_pick_place::BaxterMoveGroupInterfacePtr baxter_move_;

  ros::Publisher head_nod_topic_;
  ros::Publisher head_turn_topic_;

  // robot-specific data for generating grasps
  moveit_simple_grasps::GraspData grasp_data_;

  // allow head to move?
  bool allow_head_movements_;

  ros::NodeHandle nh_;

  SendRandom()
    : allow_head_movements_(false),
      nh_("~")
  {

    // ---------------------------------------------------------------------------------------------
    // Load grasp data specific to our robot
    if (!grasp_data_.loadRobotGraspData(nh_, "left_hand"))
      ros::shutdown();

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(grasp_data_.base_link_));
    visual_tools_->setFloorToBaseHeight(-0.9);

    // ---------------------------------------------------------------------------------------------
    // Load the move_group_inteface
    baxter_move_.reset(new baxter_pick_place::BaxterMoveGroupInterface(PLANNING_GROUP));

    // --------------------------------------------------------------------------------------------------------
    // Add objects to scene
    //createEnvironment(visual_tools_);

    // --------------------------------------------------------------------------------------------------------
    // Create publishers for stuff

    if ( allow_head_movements_ )
    {
      ROS_DEBUG_STREAM_NAMED("random_planning","Starting head nod publisher");
      head_nod_topic_ = nh_.advertise<std_msgs::Bool>("/robot/head/command_head_nod",10);

      ROS_DEBUG_STREAM_NAMED("random_planning","Starting turn head publisher");
      head_turn_topic_ = nh_.advertise<baxter_core_msgs::HeadPanCommand>("/robot/head/command_head_pan",10);
    }

    // Create the walls and tables
    //createEnvironment(visual_tools_);

    // Wait for everything to be ready
    ros::Duration(1.0).sleep();

    // Enable baxter
    if( !baxter_util_.enableBaxter() )
      return;

    // Ready the arms
    baxter_move_->positionBaxterReady();

    // Do it.
    startRoutine();

    // Shutdown
    baxter_util_.disableBaxter();
  }

  double fRand(double fMin, double fMax)
  {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
  }

  void startRoutine()
  {
    std_msgs::Bool true_command; //for nodding head
    true_command.data = true;
    baxter_core_msgs::HeadPanCommand head_command; // for turning head
    head_command.speed = 60;

    // ---------------------------------------------------------------------------------------------
    // Start the demo
    while(ros::ok())
    {
      // First look around
      if ( allow_head_movements_ )
      {
        head_command.target = fRand(-1,-0.1);
        head_turn_topic_.publish(head_command);

        ros::Duration(0.5).sleep();
        head_command.target = fRand(0.1,1.0);
        head_turn_topic_.publish(head_command);

        ros::Duration(0.5).sleep();
        head_command.target = 0.0;
        head_turn_topic_.publish(head_command);
      }

      do
      {
        ROS_INFO_STREAM_NAMED("random_planning","Planning to random target...");

        baxter_move_->positionBaxterRandom(PLANNING_GROUP);

        if ( allow_head_movements_ )
        {
          head_nod_topic_.publish(true_command);
        }
      } while(ros::ok());

      // Open grippers
      baxter_move_->openEE(true, grasp_data_);
      ros::Duration(0.25).sleep();

      // Close grippers
      baxter_move_->openEE(false, grasp_data_);
      ros::Duration(0.25).sleep();

    }

  }

}; // end class

} // nodehandle

int main(int argc, char **argv)
{
  ros::init (argc, argv, "baxter_random");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Start the pick place node
  baxter_pick_place::SendRandom();

  ros::shutdown();

  return 0;
}
