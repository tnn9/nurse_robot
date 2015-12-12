// ROS
#include <ros/ros.h>

// For visualizing things in rviz
#include <moveit_visual_tools/moveit_visual_tools.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace moveit_visual_tools
{

// Baxter specific
static const std::string EE_PARENT_LINK = "right_wrist";
static const std::string PLANNING_GROUP_NAME = "right_arm_torso_grasping";
static const std::string EE_GROUP = "right_hand";

class VisualToolsTest
{
private:

  // A shared node handle
  ros::NodeHandle nh_;

  // For visualizing things in rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // MoveIt Components
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

public:

  /**
   * \brief Constructor
   */
  VisualToolsTest()
  {
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base","/moveit_visual_tools"));
    visual_tools_->loadPlanningSceneMonitor();

    // Allow time to publish messages
    ros::spinOnce();
    ros::Duration(5.0).sleep();

    // Create pose
    geometry_msgs::Pose pose1;
    geometry_msgs::Pose pose2;

    // Test all collision shapes ----------

    ROS_INFO_STREAM_NAMED("visual_tools","Publishing Collision Mesh");
    visual_tools_->generateRandomPose(pose1);
    static const std::string package = "moveit_visual_tools";
    std::string path = "file://" + ros::package::getPath(package);
    if( path == "file://" )
      ROS_FATAL_STREAM_NAMED("visual_tools", "Unable to get " << package << " package path " );
    path.append("/resources/demo_mesh.stl");
    visual_tools_->publishCollisionMesh(pose1, "Mesh", path, rviz_visual_tools::RAND);
    ros::Duration(1.0).sleep();

    ROS_INFO_STREAM_NAMED("visual_tools","Publishing Collision Block");
    visual_tools_->generateRandomPose(pose1);
    visual_tools_->publishCollisionBlock(pose1, "Block", 0.1, rviz_visual_tools::RAND);
    ros::Duration(1.0).sleep();

    ROS_INFO_STREAM_NAMED("visual_tools","Publishing Collision Rectanglular Cuboid");
    visual_tools_->generateRandomPose(pose1);
    visual_tools_->generateRandomPose(pose2);
    visual_tools_->publishCollisionCuboid(pose1.position, pose2.position, "Rectangle", rviz_visual_tools::RAND);
    ros::Duration(1.0).sleep();

    ROS_INFO_STREAM_NAMED("visual_tools","Publishing Collision Floor");
    visual_tools_->publishCollisionFloor(0, "Floor", rviz_visual_tools::RAND);
    ros::Duration(1.0).sleep();

    ROS_INFO_STREAM_NAMED("visual_tools","Publishing Collision Cylinder");
    visual_tools_->generateRandomPose(pose1);
    visual_tools_->generateRandomPose(pose2);
    visual_tools_->publishCollisionCylinder(pose1.position, pose2.position, "Cylinder", 0.1, rviz_visual_tools::RAND);
    ros::Duration(1.0).sleep();

    // TODO: test publishCollisionGraph

    ROS_INFO_STREAM_NAMED("visual_tools","Publishing Collision Wall");
    visual_tools_->generateRandomPose(pose1);
    visual_tools_->publishCollisionWall(pose1.position.x, pose1.position.y, 0, 1, "Wall", rviz_visual_tools::RAND);
    ros::Duration(1.0).sleep();

    ROS_INFO_STREAM_NAMED("visual_tools","Publishing Collision Table");
    visual_tools_->generateRandomPose(pose1);
    visual_tools_->publishCollisionTable(pose1.position.x, pose1.position.y, 0, 0.5, 0.5, 0.5, "Table", rviz_visual_tools::RAND);
    ros::Duration(1.0).sleep();
  }

  /**
   * \brief Destructor
   */
  ~VisualToolsTest()
  {
  }

  bool loadPlanningSceneMonitor()
  {
    // Allows us to sycronize to Rviz and also publish collision objects to ourselves
    ROS_DEBUG_STREAM_NAMED("visual_tools","Loading Planning Scene Monitor");
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION, boost::shared_ptr<tf::Transformer>()));
    ros::spinOnce();
    ros::Duration(0.5).sleep();

    if (planning_scene_monitor_->getPlanningScene())
    {
      // Optional monitors to start:
      bool use_octomap_monitor = false; // this prevents a /tf warning
      planning_scene_monitor_->startWorldGeometryMonitor(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
                                                         planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
                                                         use_octomap_monitor);
      //planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
      //planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");
      planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                            "/planning_scene");
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("r2_demos","Planning scene not configured");
      return false;
    }
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    visual_tools_->setPlanningSceneMonitor(planning_scene_monitor_);

    return true;
  }

}; // end class

} // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_tools_test");
  ROS_INFO_STREAM("Visual Tools Test");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit_visual_tools::VisualToolsTest tester;

  ROS_INFO_STREAM("Shutting down.");

  return 0;
}
