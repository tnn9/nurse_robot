#ifndef BAXTER_CONTROL__BAXTER_MOVE_GROUP_INTERFACE_
#define BAXTER_CONTROL__BAXTER_MOVE_GROUP_INTERFACE_

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_simple_grasps/grasp_data.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/trajectory_processing/trajectory_tools.h> // for plan_execution
#include <moveit_msgs/MoveGroupGoal.h>
#include <moveit/kinematic_constraints/utils.h>

namespace baxter_pick_place
{

static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string NEUTRAL_POSE_NAME = "both_neutral";
static const std::string JOINT_STATE_TOPIC = "/robot/joint_states";

class BaxterMoveGroupInterface
{
public:

  // Interface with MoveIt
  boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;

  // Track the planning scene
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Trajectory execution manager
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;
  boost::shared_ptr<plan_execution::PlanExecution> plan_execution_;

  BaxterMoveGroupInterface(const std::string planning_group,
                           planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor = planning_scene_monitor::PlanningSceneMonitorPtr());

  bool positionBaxterReady();

  bool positionBaxterNeutral();

  bool positionBaxterRandom(const std::string& planning_group_name);

  /**
   * \brief open/close baxter's grippers
   * \param bool if it should be open or closed
   * \return true on success
   */
  bool openEE(bool open, const moveit_simple_grasps::GraspData& grasp_data);

  /**
   * \brief Send baxter to a named pose defined in the SRDF
   * \param pose_name - name of pose in SRDF
   * \return true if sucessful in planning and moving there
   */
  bool sendToPose(const std::string &pose_name);

  /**
   * \brief Moves the arm to a specified pose
   * \param pose - desired goal
   * \param group_name - which arm / planning group to use the pose with
   * \param visual_tools - copy of tool for publishing visual objects
   * \param planning_scene_diff - allows direct access to change the planning scene during planning
   * \return true if sucessful in planning and moving there
   */
  bool sendToPose(const geometry_msgs::Pose& pose, moveit_visual_tools::MoveItVisualToolsPtr visual_tools,
                  const moveit_msgs::PlanningScene &planning_scene_diff);

  bool sendToPose(const geometry_msgs::PoseStamped& pose,
                  moveit_visual_tools::MoveItVisualToolsPtr visual_tools, const moveit_msgs::PlanningScene &planning_scene_diff);  

  /**
   * \brief Get the current pose of the desired end effector specified in the visual_tools object
   * \param visual_tools - copy of tool for publishing visual objects
   * \return pose of end effector
   */
  geometry_msgs::Pose getCurrentPose(moveit_visual_tools::MoveItVisualToolsPtr visual_tools,
                                     const moveit_simple_grasps::GraspData& grasp_data);

  /**
   * \brief Move baxter in particular direction
   * \param
   * \param
   * \return true on success
   */
  bool moveStraight(Eigen::Vector3d approach_direction, double desired_approach_distance,
                    const moveit_simple_grasps::GraspData& grasp_data,
                    const std::string& planning_group_name, moveit_visual_tools::MoveItVisualToolsPtr visual_tools);

  /**
   * \brief
   * \param input - description
   * \param input - description
   * \return
   */
  bool executeTrajectory(const moveit_msgs::RobotTrajectory& trajectory_msg);

  /**
   * \brief
   * \param approach_direction - direction to move end effector straight
   * \param desired_approach_distance - distance the origin of a robot link needs to travel
   * \param trajectory_msg - resulting path
   * \return true on success
   */
  bool computeStraightLinePath( Eigen::Vector3d approach_direction, double desired_approach_distance,
                                moveit_msgs::RobotTrajectory& trajectory_msg, const moveit_simple_grasps::GraspData& grasp_data,
                                const std::string& planning_group_name, moveit_visual_tools::MoveItVisualToolsPtr visual_tools);

  /**
   * \brief Execute planned trajectory
   * \param trajectory_msg
   * \return true if successful
   */
  bool executeTrajectoryMsg(const moveit_msgs::RobotTrajectory& trajectory_msg);

  /**
   * \brief Hard coded poses for baxter's end effectors
   * \param side - left or right
   * \return pose of ready position
   */
  geometry_msgs::Pose getReadyPose(const std::string &side);

  bool loadPlanningSceneMonitor();

  bool convertResult(moveit::planning_interface::MoveItErrorCode& code);

  bool setRandomValidState(robot_state::RobotState &state, const robot_model::JointModelGroup* jmg);

};

typedef boost::shared_ptr<BaxterMoveGroupInterface> BaxterMoveGroupInterfacePtr;
typedef boost::shared_ptr<const BaxterMoveGroupInterface> BaxterMoveGroupInterfaceConstPtr;

} //namespace

#endif
