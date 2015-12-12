// ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <std_msgs/Float32.h>

// Boost
#include <boost/function.hpp>

namespace baxter_pick_place
{

typedef boost::function<bool()> PerturbJointFn;

class PIDTune
{
private:

  // A shared node handle
  ros::NodeHandle nh_;

  // Service Client
  ros::ServiceClient set_param_service_;

  // Subscriber
  ros::Subscriber error_sub_;

  // Service Data
  dynamic_reconfigure::ReconfigureRequest srv_req_;
  dynamic_reconfigure::ReconfigureResponse srv_resp_;
  dynamic_reconfigure::DoubleParameter double_param_;

  // Joints to tune
  std::vector<std::string> joint_names_;
  std::vector<double> joint_p_gain_;
  std::vector<double> joint_i_gain_;
  std::vector<double> joint_d_gain_;

  // Error tracking data
  std::size_t stable; // to make sure we really got the min/max
  double max_error_;
  double min_error_;
  double last_max_error_;
  double last_min_error_;
  double total_error_; // for averaging
  std::size_t total_error_count_; // for averaging
  ros::Time min_time_;
  ros::Time max_time_;

  // Single joint tuning
  double p_gain_;
  double i_gain_;
  double d_gain_;

  static const std::size_t LARGE_NUM = 99999;
public:

  // Constructor
  PIDTune()
  {
    // Start the service client
    set_param_service_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>
      ("/baxter_trajectory_controller/set_parameters");

    // Joint names
    joint_names_.push_back("right_s0");
    joint_names_.push_back("right_s1");
    joint_names_.push_back("right_e0");
    joint_names_.push_back("right_e1");
    joint_names_.push_back("right_w0");
    joint_names_.push_back("right_w1");
    joint_names_.push_back("right_w2");

    // Intial values
    double p = 2.0;
    double i = 0.0;
    double d = 0.0;

    // Fill in initial values
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      joint_p_gain_.push_back(p);
      joint_i_gain_.push_back(i);
      joint_d_gain_.push_back(d);
    }

    // Make sure trajectory controller has these initial values
    ROS_INFO_STREAM_NAMED("pid_tune","Initializing PIDs to default values:");
    sendAllGainsToController();
  }

  // Destructor
  ~PIDTune()
  {
  }

  bool sendAllGainsToController()
  {
    // Create the data structure
    dynamic_reconfigure::Config conf;

    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      // Fill in the gains
      double_param_.name = joint_names_[i] + "_kp";
      double_param_.value = joint_p_gain_[i];
      conf.doubles.push_back(double_param_);

      double_param_.name = joint_names_[i] + "_ki";
      double_param_.value = joint_p_gain_[i];
      conf.doubles.push_back(double_param_);

      double_param_.name = joint_names_[i] + "_kd";
      double_param_.value = joint_p_gain_[i];
      conf.doubles.push_back(double_param_);
    }

    // Add to request
    srv_req_.config = conf;

    // Call
    set_param_service_.call(srv_req_, srv_resp_);
    ros::spinOnce();

    //ROS_INFO_STREAM_NAMED("pid_tune","Result:\n" << srv_resp_);

    return true;
  }

  bool sendGainToController(const std::string& joint_name, double p, double i, double d)
  {
    // Create the data structure
    dynamic_reconfigure::Config conf;

    // Fill in the gains
    double_param_.name = joint_name + "_kp";
    double_param_.value = p;
    conf.doubles.push_back(double_param_);

    double_param_.name = joint_name + "_ki";
    double_param_.value = i;
    conf.doubles.push_back(double_param_);

    double_param_.name = joint_name + "_kd";
    double_param_.value = d;
    conf.doubles.push_back(double_param_);

    // Add to request
    srv_req_.config = conf;

    // Call
    set_param_service_.call(srv_req_, srv_resp_);
    ros::spinOnce();

    //ROS_INFO_STREAM_NAMED("pid_tune","Result:\n" << srv_resp_);

    return true;
  }

  bool tuneJoint(const std::string& joint_name, PerturbJointFn perturb_fn1, PerturbJointFn perturb_fn2)
  {
    // Step 1: Set I and D to zero and increase P
    p_gain_ = 2.0;
    sendGainToController(joint_name, p_gain_, 0, 0);

    // Track the error
    trackError(joint_name);

    // Lower the arm
    while(ros::ok())
    {
      perturb_fn1();
      perturb_fn2();

      // Now increase the P value
      p_gain_ += 0.25;
      sendGainToController(joint_name, p_gain_, 0, 0);
    }

  }

  void trackError(const std::string& joint_name)
  {
    stable = 0;
    min_error_ = LARGE_NUM;
    max_error_ = -LARGE_NUM;
    total_error_ = 0;
    total_error_count_ = 0;

    error_sub_ = nh_.subscribe<std_msgs::Float32>("/robot/limb/right/"+joint_name+"/trajectory/error",
                 1, &PIDTune::errorCallback, this);
  }

  void errorCallback(const std_msgs::Float32ConstPtr& msg) // ConstPtr ?
  {
    double error = msg->data;
    //    ROS_INFO_STREAM_NAMED("temp","recieved data " << msg->data);

    if( error > max_error_ )
    {
      max_error_ = error;
      max_time_ = ros::Time::now();
      min_error_ = LARGE_NUM; // reset min
      stable = 0; // reset the stability counter
    }
    else if( error < min_error_ )
    {
      min_error_ = error;
      min_time_ = ros::Time::now();
      max_error_ = -LARGE_NUM; // reset max
      stable = 0; // reset the stability counter
    }
    else
    {
      // wait here a few refreshes to make sure we got the min/max
      stable++;
      if( stable == 10 ) // do this twice per wave period
      {
        if( max_error_ == -LARGE_NUM ) // we have the last min number
        {
          last_min_error_ = min_error_;
        }
        else if( min_error_ == LARGE_NUM ) // we have the last max number
        {
          last_max_error_ = max_error_;
        }
        else
        {
          ROS_ERROR_STREAM_NAMED("temp","unexpected event with min/max");
        }
      }
    }

    if( total_error_count_ % 2 == 0 )
    {
      std::cout << "Range: [" << last_min_error_ << ", " << last_max_error_
                << "] \t Avg: " << total_error_ / total_error_count_
                << "\t Period: " << abs((max_time_ - min_time_).toSec())
                << "\t Error: " << error
                << "\t max_error: " << max_error_
                << "\t min_error: " << min_error_
                << "\t p_gain: " << p_gain_ << "\n";
    }


    total_error_ += error;
    total_error_count_++;
  }

}; // end class

} // end namespace
