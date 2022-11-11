// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/model.hpp>

#include <proxsuite/proxqp/dense/dense.hpp>

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace franka_example_controllers {

class WorkflowTestController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  pinocchio::Model model;
  pinocchio::Data data;

  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  ros::Duration elapsed_time_;
  std::array<double, 7> initial_pose_{};
};

}  // namespace franka_example_controllers
