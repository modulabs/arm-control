// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <trajectory_interface/quintic_spline_segment.h>
#include <time_delay_controller.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>

namespace time_delay_controller
{
  typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                                 hardware_interface::EffortJointInterface>
          JointTrajectoryController;
}

PLUGINLIB_EXPORT_CLASS(time_delay_controller::JointTrajectoryController,   controller_interface::ControllerBase)
