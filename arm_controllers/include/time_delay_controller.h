#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <boost/scoped_ptr.hpp>

#include <std_msgs/Float64MultiArray.h>

#include "arm_controllers/TimedelayControllerParamsConfig.h"

#define SaveDataMax 8
#define JointMax 6
#define PI 3.141592
#define D2R PI/180.0
#define R2D 180.0/PI

namespace Controllers
{
class time_delay_controller{};
}

template <class State>
class ClosedLoopHardwareInterfaceAdapter<State, Controllers::time_delay_controller>
{
  class Gains
  {
  public:
    Gains()
        : mbar_(0.0), ramda_(0.0)
    {
    }

    void setGains(double mbar, double ramda)
    {
      mbar_ = mbar;
      ramda_ = ramda;
    }

    void setGains()
    {
    }

    void getGains(double &mbar, double &ramda)
    {
      mbar = mbar_;
      ramda = ramda_;
    }

    void initDynamicReconfig(const ros::NodeHandle &node)
    {
      ROS_INFO("Init dynamic reconfig in namespace %s", node.getNamespace().c_str());

      // Start dynamic reconfigure server
      param_reconfig_server_.reset(new DynamicReconfigServer(param_reconfig_mutex_, node));
      dynamic_reconfig_initialized_ = true;

      // Set Dynamic Reconfigure's gains to Pid's values
      updateDynamicReconfig();

      // Set callback
      param_reconfig_callback_ = boost::bind(&Gains::dynamicReconfigCallback, this, _1, _2);
      param_reconfig_server_->setCallback(param_reconfig_callback_);
    }

    void updateDynamicReconfig()
    {
      // Make sure dynamic reconfigure is initialized
      if (!dynamic_reconfig_initialized_)
        return;

      // Get starting values
      arm_controllers::TimedelayControllerParamsConfig config;

      // Get starting values
      getGains(config.mbar, config.ramda);

      updateDynamicReconfig(config);
    }

    void updateDynamicReconfig(arm_controllers::TimedelayControllerParamsConfig config)
    {
      // Make sure dynamic reconfigure is initialized
      if (!dynamic_reconfig_initialized_)
        return;

      // Set starting values, using a shared mutex with dynamic reconfig
      param_reconfig_mutex_.lock();
      param_reconfig_server_->updateConfig(config);
      param_reconfig_mutex_.unlock();
    }

    void dynamicReconfigCallback(arm_controllers::TimedelayControllerParamsConfig &config, uint32_t /*level*/)
    {
      ROS_DEBUG_STREAM_NAMED("ATDC", "Dynamics reconfigure callback recieved.");

      // Set the gains
      setGains(config.mbar, config.ramda);
    }

    double mbar_;
    double ramda_;

  private:
    // Dynamics reconfigure
    bool dynamic_reconfig_initialized_;
    typedef dynamic_reconfigure::Server<arm_controllers::TimedelayControllerParamsConfig> DynamicReconfigServer;
    boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
    DynamicReconfigServer::CallbackType param_reconfig_callback_;

    boost::recursive_mutex param_reconfig_mutex_;
  };

public:
  ClosedLoopHardwareInterfaceAdapter() : joint_handles_ptr_(0) {}

  bool init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& controller_nh)
  {
    // Store pointer to joint handles
    joint_handles_ptr_ = &joint_handles;
    n_joints_ = joint_handles.size();

    if (!controller_nh.getParam("joints", joint_names_))
    {
      ROS_ERROR("Could not find joint name");
      return false;
    }

    // command and state
    tau_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
    tau_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
    q_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
    qdot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
    qdot_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
    qddot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);

    q_init_.data = Eigen::VectorXd::Zero(n_joints_);
    q_.data = Eigen::VectorXd::Zero(n_joints_);
    qdot_.data = Eigen::VectorXd::Zero(n_joints_);
    qdot_old_.data = Eigen::VectorXd::Zero(n_joints_);
    qddot_.data = Eigen::VectorXd::Zero(n_joints_);
    s_.data = Eigen::VectorXd::Zero(n_joints_);

    q_error_.data = Eigen::VectorXd::Zero(n_joints_);
    q_error_dot_.data = Eigen::VectorXd::Zero(n_joints_);

    tde_.data = Eigen::VectorXd::Zero(n_joints_);
    ded_.data = Eigen::VectorXd::Zero(n_joints_);

    dt_ = 0.0;
    time_ = 0.0;

    // gains
		mbar_.resize(n_joints_);
		ramda_.resize(n_joints_);

    for (size_t i = 0; i < n_joints_; i++)
    {
      gains_.push_back(new Gains());
      std::string si = boost::lexical_cast<std::string>(i + 1);
      if (!controller_nh.getParam("/time_delay_controller/joint" + si + "/tdc/mbar", gains_[i]->mbar_))
      {
        ROS_ERROR("Cannot find tdc/mbar gain");
        return false;
      }

      if (!controller_nh.getParam("/time_delay_controller/joint" + si + "/tdc/r", gains_[i]->ramda_))
      {
        ROS_ERROR("Cannot find tdc/r gain");
        return false;
      }

      gains_[i]->initDynamicReconfig(ros::NodeHandle(controller_nh, "gains/" + joint_names_[i]));
    }

    pub_SaveData_Joint1_ = controller_nh.advertise<std_msgs::Float64MultiArray>("SaveData_Joint1", 1000);
    pub_SaveData_Joint2_ = controller_nh.advertise<std_msgs::Float64MultiArray>("SaveData_Joint2", 1000);
    pub_SaveData_Joint3_ = controller_nh.advertise<std_msgs::Float64MultiArray>("SaveData_Joint3", 1000);
    pub_SaveData_Joint4_ = controller_nh.advertise<std_msgs::Float64MultiArray>("SaveData_Joint4", 1000);
    pub_SaveData_Joint5_ = controller_nh.advertise<std_msgs::Float64MultiArray>("SaveData_Joint5", 1000);
    pub_SaveData_Joint6_ = controller_nh.advertise<std_msgs::Float64MultiArray>("SaveData_Joint6", 1000);

    return true;
  }

  void starting(const ros::Time & /*time*/)
  {
    if (!joint_handles_ptr_)
    {
      return;
    }

    for (size_t i = 0; i < n_joints_; i++)
    {
      (*joint_handles_ptr_)[i].setCommand(0.0);
    }    

    ROS_INFO("Starting Time Delay Controller");
  }

  void stopping(const ros::Time& /*time*/) {}

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& period,
                     const State&         desired_state,
                     const State&         state_error)
  {
    const unsigned int n_joints = joint_handles_ptr_->size();

    // Preconditions
    if (!joint_handles_ptr_)
      return;
    assert(n_joints == state_error.position.size());
    assert(n_joints == state_error.velocity.size());

    dt_ = period.toSec();

    // get joint states
    for (size_t i = 0; i < n_joints_; i++)
    {
      q_(i) = (*joint_handles_ptr_)[i].getPosition();
      qdot_(i) = (*joint_handles_ptr_)[i].getVelocity();
      qddot_(i) = (qdot_(i) - qdot_old_(i)) / dt_;
      qdot_old_(i) = qdot_(i);
    }
    
    for (size_t i = 0; i < n_joints_; i++)
    {
      q_error_(i) = desired_state.position[i] - q_(i);
      q_error_dot_(i) = desired_state.velocity[i] - qdot_(i);
      qddot_cmd_(i) = (desired_state.velocity[i] - qdot_cmd_old_(i)) / dt_;
      qdot_cmd_old_(i) = desired_state.velocity[i];
    }

//    task_tunning();

    double db = 0.001;

    for (size_t i = 0; i < n_joints_; i++)
    {
      ded_(i) = qddot_cmd_(i) + 2.0 * gains_[i]->ramda_ * q_error_dot_(i) + gains_[i]->ramda_ * gains_[i]->ramda_ * q_error_(i);
      tde_(i) = tau_cmd_old_(i) - gains_[i]->mbar_* qddot_(i);
      tau_cmd_(i) = gains_[i]->mbar_ * ded_(i) + tde_(i);
    }

    tau_cmd_old_.data = tau_cmd_.data;

    for (int i = 0; i < n_joints_; i++)
    {
      (*joint_handles_ptr_)[i].setCommand(tau_cmd_(i));
    }

    data_save();

    time_ = time_ + dt_;
  }

  void task_tunning()
  {
    double mag = 45.0 * D2R;
    double freq = 0.3;

    // get joint states
    for (size_t i = 0; i < n_joints_; i++)
    {
      q_cmd_(i) = q_init_(i) + mag * sin(2 * PI * freq * time_);
      qdot_cmd_(i) = mag * 2 * PI * freq * cos(2 * PI * freq * time_);
      qddot_cmd_(i) = -mag * 2 * PI * freq * 2 * PI * freq * sin(2 * PI * freq * time_);

      q_(i) = (*joint_handles_ptr_)[i].getPosition();
      qdot_(i) = (*joint_handles_ptr_)[i].getVelocity();
      qddot_(i) = (qdot_(i) - qdot_old_(i)) / dt_;
      qdot_old_(i) = qdot_(i);
    }

    for (size_t i = 0; i < n_joints_; i++)
    {
      q_error_(i) = q_cmd_(i) - q_(i);
      q_error_dot_(i) = qdot_cmd_(i) - qdot_(i);
    }
  }

  void data_save()
  {
    SaveData_Joint1_[0] = time_;
    SaveData_Joint1_[1] = q_(0) * R2D;
    SaveData_Joint1_[2] = qdot_(0) * R2D;
    SaveData_Joint1_[3] = q_cmd_(0) * R2D;
    SaveData_Joint1_[4] = qdot_cmd_(0) * R2D;
    SaveData_Joint1_[5] = q_error_(0) * R2D;
    SaveData_Joint1_[6] = q_error_dot_(0) * R2D;
    SaveData_Joint1_[7] = tau_cmd_(0);

    SaveData_Joint2_[0] = time_;
    SaveData_Joint2_[1] = q_(1) * R2D;
    SaveData_Joint2_[2] = qdot_(1) * R2D;
    SaveData_Joint2_[3] = q_cmd_(1) * R2D;
    SaveData_Joint2_[4] = qdot_cmd_(1) * R2D;
    SaveData_Joint2_[5] = q_error_(1) * R2D;
    SaveData_Joint2_[6] = q_error_dot_(1) * R2D;
    SaveData_Joint2_[7] = tau_cmd_(1);

    SaveData_Joint3_[0] = time_;
    SaveData_Joint3_[1] = q_(2) * R2D;
    SaveData_Joint3_[2] = qdot_(2) * R2D;
    SaveData_Joint3_[3] = q_cmd_(2) * R2D;
    SaveData_Joint3_[4] = qdot_cmd_(2) * R2D;
    SaveData_Joint3_[5] = q_error_(2) * R2D;
    SaveData_Joint3_[6] = q_error_dot_(2) * R2D;
    SaveData_Joint3_[7] = tau_cmd_(2);

    SaveData_Joint4_[0] = time_;
    SaveData_Joint4_[1] = q_(3) * R2D;
    SaveData_Joint4_[2] = qdot_(3) * R2D;
    SaveData_Joint4_[3] = q_cmd_(3) * R2D;
    SaveData_Joint4_[4] = qdot_cmd_(3) * R2D;
    SaveData_Joint4_[5] = q_error_(3) * R2D;
    SaveData_Joint4_[6] = q_error_dot_(3) * R2D;
    SaveData_Joint4_[7] = tau_cmd_(3);

    SaveData_Joint5_[0] = time_;
    SaveData_Joint5_[1] = q_(4) * R2D;
    SaveData_Joint5_[2] = qdot_(4) * R2D;
    SaveData_Joint5_[3] = q_cmd_(4) * R2D;
    SaveData_Joint5_[4] = qdot_cmd_(4) * R2D;
    SaveData_Joint5_[5] = q_error_(4) * R2D;
    SaveData_Joint5_[6] = q_error_dot_(4) * R2D;
    SaveData_Joint5_[7] = tau_cmd_(4);

    SaveData_Joint6_[0] = time_;
    SaveData_Joint6_[1] = q_(5) * R2D;
    SaveData_Joint6_[2] = qdot_(5) * R2D;
    SaveData_Joint6_[3] = q_cmd_(5) * R2D;
    SaveData_Joint6_[4] = qdot_cmd_(5) * R2D;
    SaveData_Joint6_[5] = q_error_(5) * R2D;
    SaveData_Joint6_[6] = q_error_dot_(5) * R2D;
    SaveData_Joint6_[7] = tau_cmd_(5);

    msg_SaveData_Joint1_.data.clear();
    msg_SaveData_Joint2_.data.clear();
    msg_SaveData_Joint3_.data.clear();
    msg_SaveData_Joint4_.data.clear();
    msg_SaveData_Joint5_.data.clear();
    msg_SaveData_Joint6_.data.clear();

    for (size_t i = 0; i < SaveDataMax; i++)
    {
      msg_SaveData_Joint1_.data.push_back(SaveData_Joint1_[i]);
      msg_SaveData_Joint2_.data.push_back(SaveData_Joint2_[i]);
      msg_SaveData_Joint3_.data.push_back(SaveData_Joint3_[i]);
      msg_SaveData_Joint4_.data.push_back(SaveData_Joint4_[i]);
      msg_SaveData_Joint5_.data.push_back(SaveData_Joint5_[i]);
      msg_SaveData_Joint6_.data.push_back(SaveData_Joint6_[i]);
    }

    pub_SaveData_Joint1_.publish(msg_SaveData_Joint1_);
    pub_SaveData_Joint2_.publish(msg_SaveData_Joint2_);
    pub_SaveData_Joint3_.publish(msg_SaveData_Joint3_);
    pub_SaveData_Joint4_.publish(msg_SaveData_Joint4_);
    pub_SaveData_Joint5_.publish(msg_SaveData_Joint5_);
    pub_SaveData_Joint6_.publish(msg_SaveData_Joint6_);
  }

private:
  std::vector<hardware_interface::JointHandle> *joint_handles_ptr_;
  std::vector<std::string> joint_names_;

  unsigned int n_joints_;

  // tdc gain
  KDL::JntArray mbar_, ramda_;
  KDL::JntArray ded_, tde_;

  // cmd, state
  KDL::JntArray q_init_;
  KDL::JntArray tau_cmd_, tau_cmd_old_;
  KDL::JntArray q_cmd_, qdot_cmd_, qdot_cmd_old_, qddot_cmd_;
  KDL::JntArray q_, qdot_, qdot_old_, qddot_;
  KDL::JntArray q_error_, q_error_dot_;
  KDL::JntArray s_;

  // gain
  std::vector<Gains *> gains_;

  double dt_, time_;

  // topic
  double SaveData_Joint1_[SaveDataMax];
  ros::Publisher pub_SaveData_Joint1_;
  std_msgs::Float64MultiArray msg_SaveData_Joint1_;

  double SaveData_Joint2_[SaveDataMax];
  ros::Publisher pub_SaveData_Joint2_;
  std_msgs::Float64MultiArray msg_SaveData_Joint2_;

  double SaveData_Joint3_[SaveDataMax];
  ros::Publisher pub_SaveData_Joint3_;
  std_msgs::Float64MultiArray msg_SaveData_Joint3_;

  double SaveData_Joint4_[SaveDataMax];
  ros::Publisher pub_SaveData_Joint4_;
  std_msgs::Float64MultiArray msg_SaveData_Joint4_;

  double SaveData_Joint5_[SaveDataMax];
  ros::Publisher pub_SaveData_Joint5_;
  std_msgs::Float64MultiArray msg_SaveData_Joint5_;

  double SaveData_Joint6_[SaveDataMax];
  ros::Publisher pub_SaveData_Joint6_;
  std_msgs::Float64MultiArray msg_SaveData_Joint6_;
};

template <class State>
class HardwareInterfaceAdapter<hardware_interface::EffortJointInterface, State, Controllers::time_delay_controller> : public ClosedLoopHardwareInterfaceAdapter<State, Controllers::time_delay_controller>
{};
