#include <franka_example_controllers/joint_pd_test_controller.h>
#include <franka/robot_state.h>
#include <franka_example_controllers/pseudo_inversion.h>

#include <cmath>
#include <optional>

#include <controller_interface/controller_base.h> 
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>   

namespace franka_example_controllers {

bool JointPDTestController::init(hardware_interface::RobotHW* robot_hw,
                                          ros::NodeHandle& node_handle) {
  // check if got arm_id
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("JointPDTestController: Could not read parameter arm_id");
    return false;
  }
  
  // check if got joint_names
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPDTestController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPDTestController: Wrong number of joint names, got " << joint_names.size() << " instead of 7 names!");
    return false;
  }
  
  // check if model_handle_ works
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointPDTestController: Error getting model interface from hardware");
    return false;
  }

  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointPDTestController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  // check if state_handle_ works
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointPDTestController: Error getting state interface from hardware");
    return false;
  }
  
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointPDTestController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  // check if joint_handles_ got each joint (crucial step of avoiding could not switch controller error)
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointPDTestController: Error getting effort joint interface from hardware");
    return false;
  }
  
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointPDTestController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR(
        "JointPDTestController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR(
        "JointPDTestController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  read_gains(node_handle);

  return true;
}

void JointPDTestController::starting(const ros::Time& /* time */) {
  // get intial robot state
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_init(initial_state.q.data());

  // get current end-effector position and orientation
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  p_target = transform.translation();
  R_target = transform.rotation();
  dP_target << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  
  // initialize clock
  controlller_clock = 0.0;

  Eigen::Map<Eigen::Matrix<double, 7, 1>> k_gains_array(k_gains_.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> d_gains_array(d_gains_.data());

  Kp = k_gains_array.array().matrix().asDiagonal();
  Kd = d_gains_array.array().matrix().asDiagonal();
}

void JointPDTestController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  // update controller clock
  controlller_clock += period.toSec();

  // get joint angles and angular velocities
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

  // measure end-effector position and orientation
  Eigen::Affine3d measured_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  p_measured = measured_transform.translation();
  R_measured = measured_transform.rotation();

  // get end-effector jacobian
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

  // compute orientation error with targets
  Eigen::Matrix<double, 3, 3> R_error = R_target * R_measured.transpose();
  Eigen::AngleAxisd AngleAxisErr(R_error);
  Eigen::Vector3d rotvec_err = AngleAxisErr.axis() * AngleAxisErr.angle();

  double amplitude = 0.3;
  double half_period = 3.0;

  // compute new p_target along the y-axis
  p_target[1] = std::sin(M_PI * controlller_clock / half_period) * amplitude;

  // compute new dP_target along the y-axis
  dP_target[1] = (M_PI / half_period) * std::cos(M_PI * controlller_clock / half_period) * amplitude;

  // compute positional error
  Eigen::Matrix<double, 6, 1> P_error;
  P_error << p_target - p_measured, rotvec_err;

  // compute pseudo-inverse of Jacobian
  pseudoInverse(jacobian, pinv_jacobian);

  // compute joint target
  delta_q_target = pinv_jacobian * P_error;

  // get mass matrix
  std::array<double, 49> mass_array = model_handle_->getMass();
  Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass_array.data());

  // get Coriolis and centrifugal terms
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

  // get gravitational terms
  std::array<double, 7> gravitational_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravitational(gravitational_array.data());

  auto ddq_cmd = Kp * delta_q_target + Kd * (pinv_jacobian * dP_target - dq);

  // compute joint torque
  // torques = Kp * delta_q_target + Kd * (pinv_jacobian * dP_target - dq) - dq_gain * dq;
  torques = M * ddq_cmd + coriolis;
  // torques = M * ddq_cmd;

  // Saturate torque rate to avoid discontinuities
  torques << saturateTorqueRate(torques, tau_J_d);

  // set torque
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(torques[i]);
  }

  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> _tau_J_d(robot_state.tau_J_d.data());

  // ROS_INFO_STREAM("Positional Error: " << P_error.transpose());
  ROS_INFO_STREAM("=========================================");
  ROS_INFO_STREAM("Torque Error     : " << (tau_J - tau_J_d - gravitational).transpose());
  ROS_INFO_STREAM("ddq Commanded    : " << ddq_cmd.transpose());
  ROS_INFO_STREAM("Torque Commanded : " << torques.transpose());
  ROS_INFO_STREAM("Torque Measured  : " << tau_J.transpose());
  ROS_INFO_STREAM("Torque Desired   : " << tau_J_d.transpose());
}

Eigen::Matrix<double, 7, 1> JointPDTestController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

bool JointPDTestController::read_gains(ros::NodeHandle& node_handle) {
  // check if got p_gain
  if (!node_handle.getParam("p_gain", p_gain)) {
    ROS_ERROR_STREAM("JointPDTestPinController: Could not read parameter p_gain");
    return false;
  }

  // check if got d_gain
  if (!node_handle.getParam("d_gain", d_gain)) {
    ROS_ERROR_STREAM("JointPDTestPinController: Could not read parameter d_gain");
    return false;
  }

  // check if got dq_gain
  if (!node_handle.getParam("dq_gain", dq_gain)) {
    ROS_ERROR_STREAM("JointPDTestPinController: Could not read parameter dq_gain");
    return false;
  }

  return true;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPDTestController, controller_interface::ControllerBase)