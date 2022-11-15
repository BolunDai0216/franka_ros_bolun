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

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

#include <proxsuite/proxqp/dense/dense.hpp>
#include <proxsuite/proxqp/utils/random_qp_problems.hpp>

// #include <eigen/Core>
// #include <Eigen/Geometry> 

namespace franka_example_controllers {

class ProxsuiteController : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface, 
                                                                                  hardware_interface::EffortJointInterface, 
                                                                                  franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void alpha_func(const double& t);
  void solve_qp(void);
  void get_qp_parameters(const Eigen::Matrix<double, 7, 1>& q, 
                         const Eigen::Matrix<double, 7, 1>& dq);

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  // pinocchio model & data
  pinocchio::Model model;
  pinocchio::Data data;

  // interface with franka_hw
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  const double delta_tau_max_{1.0};

  // initial position and orientation
  Eigen::Matrix<double, 3, 1> p_start;
  Eigen::Matrix<double, 3, 3> R_start;

  // initial orientation error
  Eigen::Matrix<double, 3, 3> R_error;
  Eigen::Vector3d orientation_error_axis;
  Eigen::AngleAxisd::Scalar orientation_error_angle;

  // target position and orientation at each time step
  Eigen::Matrix<double, 3, 1> p_target;
  Eigen::Matrix<double, 3, 1> v_target;
  Eigen::Matrix<double, 3, 1> a_target;
  Eigen::Matrix<double, 3, 3> R_target;
  Eigen::AngleAxisd::Scalar angle_target;
  Eigen::Vector3d w_target;
  Eigen::Vector3d dw_target;

  // measured position and orientation
  Eigen::Matrix<double, 3, 1> p_current;
  Eigen::Matrix<double, 3, 3> R_current;

  // terminal target position and orientation
  Eigen::Matrix<double, 3, 1> p_end;
  Eigen::Matrix<double, 3, 3> R_end;

  // errors and targets computed at each time step
  Eigen::Matrix<double, 6, 1> P_err; 
  Eigen::Matrix<double, 6, 1> dP_target;
  Eigen::Matrix<double, 6, 1> ddP_target;

  // pseudo-inverse
  Eigen::MatrixXd pJ_EE;

  // clock only for task controller
  double controlller_clock;

  // movement duration of a segment
  double movement_duration;

  // alpha values
  double alpha;
  double dalpha;
  double ddalpha;

  // end-effector frame id in pinocchio
  int ee_frame_id;

  // end-effector Jacobian
  Eigen::Matrix<double, 6, 7> J;

  // time deriative of end-effector Jacobian 
  Eigen::Matrix<double, 6, 7> dJ;

  // applied torque
  Eigen::Matrix<double, 7, 1> q_desired;
  Eigen::Matrix<double, 7, 1> torques;

  // define Kp and Kd
  Eigen::Matrix<double, 6, 6> Kp;
  Eigen::Matrix<double, 6, 6> Kd;

  // create QP
  // dense::QP<double> qp(14, 7, 0);

  // QP initialization indicator
  bool qp_initialized;

  // define QP parameters
  Eigen::Matrix<double, 7, 7> qp_H;
  Eigen::Matrix<double, 7, 1> qp_g;
  Eigen::Matrix<double, 7, 1> q_nominal;
};

}  // namespace franka_example_controllers