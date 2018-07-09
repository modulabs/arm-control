// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <trajectory_interface/quintic_spline_segment.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>

namespace Controllers
{
typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                               hardware_interface::EffortJointInterface,
                                                               Controllers::computed_torque_controller>
    ComputedTorqueController;

typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                               hardware_interface::EffortJointInterface,
                                                               Controllers::gravity_comp_controller>
    GravityCompController;

typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                               hardware_interface::EffortJointInterface,
                                                               Controllers::time_delay_controller>
    TimeDelayController;            
}

PLUGINLIB_EXPORT_CLASS(Controllers::ComputedTorqueController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(Controllers::GravityCompController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(Controllers::TimeDelayController, controller_interface::ControllerBase)