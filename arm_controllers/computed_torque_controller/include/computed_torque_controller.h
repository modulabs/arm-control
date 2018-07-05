#ifndef JOINT_TRAJECTORY_CONTROLLER_HARDWARE_INTERFACE_ADAPTER_H
#define JOINT_TRAJECTORY_CONTROLLER_HARDWARE_INTERFACE_ADAPTER_H

#include <cassert>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include <ros/node_handle.h>
#include <ros/time.h>

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>

#include <urdf/model.h>

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>

template <class HardwareInterface, class State>
class HardwareInterfaceAdapter
{
public:
  bool init(std::vector<typename HardwareInterface::ResourceHandleType>& /*joint_handles*/, ros::NodeHandle& /*controller_nh*/)
  {
    return false;
  }

  void starting(const ros::Time& /*time*/) {}
  void stopping(const ros::Time& /*time*/) {}

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& /*period*/,
                     const State&         /*desired_state*/,
                     const State&         /*state_error*/) {}
};

template <class State>
class ClosedLoopHardwareInterfaceAdapter
{
public:
  ClosedLoopHardwareInterfaceAdapter() : joint_handles_ptr_(0) {}

  bool init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& controller_nh)
  {
    joint_handles_ptr_ = &joint_handles;
    n_joints_ = joint_handles_ptr_->size();

    pids_.resize(joint_handles.size());
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      ros::NodeHandle joint_nh(controller_nh, std::string("gains/") + joint_handles[i].getName());

      pids_[i].reset(new control_toolbox::Pid());
      if (!pids_[i]->init(joint_nh))
      {
        ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
        return false;
      }
    }

    urdf::Model urdf;
    if (!urdf.initParam("robot_description"))
    {
      ROS_ERROR("Failed to parse urdf file");
      return false;
    }
    else
    {
      ROS_INFO("Found robot_description");
    }

    if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
    {
      ROS_ERROR("Failed to construct kdl tree");
      return false;
    }
    else
    {
      ROS_INFO("Constructed kdl tree");
    }

    std::string root_name, tip_name;
    if (!controller_nh.getParam("root_link", root_name))
    {
      ROS_ERROR("Could not find root link name");
      return false;
    }
    if (!controller_nh.getParam("tip_link", tip_name))
    {
      ROS_ERROR("Could not find tip link name");
      return false;
    }
    if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
    {
      ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
      ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
      ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
      ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
      ROS_ERROR_STREAM("  The segments are:");

      KDL::SegmentMap segment_map = kdl_tree_.getSegments();
      KDL::SegmentMap::iterator it;

      for (it = segment_map.begin(); it != segment_map.end(); it++)
        ROS_ERROR_STREAM("    " << (*it).first);

      return false;
    }
    else
    {
      ROS_INFO("Got kdl chain");
    }

    gravity_ = KDL::Vector::Zero(); 
    gravity_(2) = -9.81;         

    id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

    q_.data = Eigen::VectorXd::Zero(n_joints_);
    qdot_.data = Eigen::VectorXd::Zero(n_joints_);

    qd_.data = Eigen::VectorXd::Zero(n_joints_);
    qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
    qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
    qd_old_.data = Eigen::VectorXd::Zero(n_joints_);
    qd_dot_old_.data = Eigen::VectorXd::Zero(n_joints_);

    aux_.data = Eigen::VectorXd::Zero(n_joints_);
    comp_.data = Eigen::VectorXd::Zero(n_joints_);
    tau_pid_.data = Eigen::VectorXd::Zero(n_joints_);
    tau_d_.data = Eigen::VectorXd::Zero(n_joints_);

    tau_damping_.data = Eigen::VectorXd::Zero(n_joints_);
    tau_coulomb_.data = Eigen::VectorXd::Zero(n_joints_);
    tau_friction_.data = Eigen::VectorXd::Zero(n_joints_);

    M_.resize(kdl_chain_.getNrOfJoints());
    C_.resize(kdl_chain_.getNrOfJoints());
    G_.resize(kdl_chain_.getNrOfJoints());

    return true;
  }

  void starting(const ros::Time& /*time*/)
  {
    t = 0.0;

    if (!joint_handles_ptr_) {return;}

    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      pids_[i]->reset();
      (*joint_handles_ptr_)[i].setCommand(0.0);
    }

    ROS_INFO("Starting Computed Torque Controller");
  }

  void stopping(const ros::Time& /*time*/) {}

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& period,
                     const State&         desired_state,
                     const State&         state_error)
  {
    if (!joint_handles_ptr_)
      return;
    assert(n_joints_ == state_error.position.size());
    assert(n_joints_ == state_error.velocity.size());

    t = t + period.toSec();

    for (size_t i = 0; i < n_joints_; i++)
    {
      q_(i) = (*joint_handles_ptr_)[i].getPosition();
      qdot_(i) = (*joint_handles_ptr_)[i].getVelocity();
    }

    for (size_t i = 0; i < n_joints_; i++)
    {
      qd_(i) = desired_state.position[i];
      qd_dot_(i) = desired_state.velocity[i];
      qd_ddot_(i) = (qd_dot_(i) - qd_dot_old_(i)) / period.toSec();
      qd_dot_old_(i) =  qd_dot_(i);
    }

    id_solver_->JntToMass(q_, M_);
    id_solver_->JntToCoriolis(q_, qdot_, C_);
    id_solver_->JntToGravity(q_, G_);

    for (size_t i = 0; i < n_joints_; i++)
    {
      tau_damping_(i) = 1.0 * qdot_(i);
      tau_coulomb_(i) = 1.0 * KDL::sign(qdot_(i));
    }

    tau_friction_.data = tau_damping_.data + tau_coulomb_.data;

    for (size_t i = 0; i < n_joints_; i++)
    {
      tau_pid_(i) = pids_[i]->computeCommand(state_error.position[i], state_error.velocity[i], period);
    }

    aux_.data = M_.data * (qd_ddot_.data + tau_pid_.data);
    comp_.data = C_.data + G_.data + tau_friction_.data;
    tau_d_.data = aux_.data + comp_.data;

    for (size_t i = 0; i < n_joints_; i++)
    {
      (*joint_handles_ptr_)[i].setCommand(tau_d_(i));
    }
  }

private:
  typedef boost::shared_ptr<control_toolbox::Pid> PidPtr;
  std::vector<PidPtr> pids_;

  std::vector<hardware_interface::JointHandle> *joint_handles_ptr_;

  KDL::Tree kdl_tree_;  
  KDL::Chain kdl_chain_;

  KDL::JntSpaceInertiaMatrix M_;
  KDL::JntArray C_;   
  KDL::JntArray G_;       
  KDL::Vector gravity_;

  boost::scoped_ptr<KDL::ChainDynParam> id_solver_; 

  KDL::JntArray q_, qdot_;
  KDL::JntArray qd_, qd_dot_, qd_ddot_;
  KDL::JntArray qd_old_, qd_dot_old_;

  KDL::JntArray aux_;
  KDL::JntArray comp_;
  KDL::JntArray tau_pid_;
  KDL::JntArray tau_d_;

  KDL::JntArray tau_damping_, tau_coulomb_, tau_friction_;

  double t;
  unsigned int n_joints_; 
};

template <class State>
class HardwareInterfaceAdapter<hardware_interface::EffortJointInterface, State> : public ClosedLoopHardwareInterfaceAdapter<State>
{};

#endif // header guard
