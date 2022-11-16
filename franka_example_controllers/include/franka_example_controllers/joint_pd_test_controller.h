#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

namespace franka_example_controllers {

class JointPDTestController : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface, 
                                                                                    hardware_interface::EffortJointInterface, 
                                                                                    franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  bool read_gains(ros::NodeHandle& node_handle);

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  // interface with franka_hw
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // clock only for controller
  double controlller_clock;

  // max torque rate
  const double delta_tau_max_{1.0};

  // joint targets
  Eigen::Matrix<double, 7, 1> delta_q_target;
  Eigen::Matrix<double, 7, 1> dq_target;

  // applied torque
  Eigen::Matrix<double, 7, 1> torques;

  // fixed rotation matrix target
  Eigen::Matrix<double, 3, 3> R_target;

  // changing position vector target
  Eigen::Matrix<double, 3, 1> p_target;
  Eigen::Matrix<double, 6, 1> dP_target;

  // measured end-effector configuration
  Eigen::Matrix<double, 3, 1> p_measured;
  Eigen::Matrix<double, 3, 3> R_measured;

  // pseudo-inverse
  Eigen::MatrixXd pinv_jacobian;

  // define gains for pd controll
  double p_gain;
  double d_gain;
  double dq_gain;

  Eigen::Matrix<double, 7, 7> Kp;
  Eigen::Matrix<double, 7, 7> Kd;

  std::vector<double> k_gains_;
  std::vector<double> d_gains_;
};

}  // namespace franka_example_controllers