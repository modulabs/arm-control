// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <trajectory_interface/quintic_spline_segment.h>
#include <computed_torque_controller.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>

namespace computed_torque_controller
{
  typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                                 hardware_interface::EffortJointInterface>
          JointTrajectoryController;
}

PLUGINLIB_EXPORT_CLASS(computed_torque_controller::JointTrajectoryController,   controller_interface::ControllerBase)
