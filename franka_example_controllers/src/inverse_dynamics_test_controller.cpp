#include <franka_example_controllers/inverse_dynamics_test_controller.h>
#include <franka/robot_state.h>
#include <franka_example_controllers/pseudo_inversion.h>

#include <cmath>
#include <optional>

#include <controller_interface/controller_base.h> 
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>   

namespace pin = pinocchio;

namespace franka_example_controllers {

bool InverseDynamicsTestController::init(hardware_interface::RobotHW* robot_hw,
                                          ros::NodeHandle& node_handle) {
  // check if got arm_id
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("InverseDynamicsTestController: Could not read parameter arm_id");
    return false;
  }
  
  // check if got joint_names
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("InverseDynamicsTestController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("InverseDynamicsTestController: Wrong number of joint names, got " << joint_names.size() << " instead of 7 names!");
    return false;
  }
  
  // check if model_handle_ works
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "InverseDynamicsTestController: Error getting model interface from hardware");
    return false;
  }

  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "InverseDynamicsTestController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  // check if state_handle_ works
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "InverseDynamicsTestController: Error getting state interface from hardware");
    return false;
  }
  
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "InverseDynamicsTestController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  // check if joint_handles_ got each joint (crucial step of avoiding could not switch controller error)
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "InverseDynamicsTestController: Error getting effort joint interface from hardware");
    return false;
  }
  
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "InverseDynamicsTestController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 6) {
    ROS_ERROR(
        "JointPDTestController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 6) {
    ROS_ERROR(
        "JointPDTestController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("td_gains", td_gains_) || td_gains_.size() != 7) {
    ROS_ERROR(
        "JointPDTestController:  Invalid or no td_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  read_gains(node_handle);

  // build pin_robot from urdf
  std::string urdf_filename = "/home/bolun/bolun_ws/src/franka_ros_bolun/franka_example_controllers/fr3.urdf";
  pin::urdf::buildModel(urdf_filename, model);
  data = pin::Data(model);

  return true;
}

void InverseDynamicsTestController::starting(const ros::Time& /* time */) {
  // get intial robot state
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_init(initial_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_init(initial_state.dq.data());

  // update pinocchio robot model
  pin::forwardKinematics(model, data, q_init, dq_init);
  pin::computeJointJacobians(model, data, q_init); 
  pin::updateFramePlacements(model, data);

  // define end-effector frame id in pinocchio
  ee_frame_id = model.getFrameId("fr3_hand_tcp");

  // get current end-effector position and orientation
  p_target = data.oMf[ee_frame_id].translation();
  R_target = data.oMf[ee_frame_id].rotation();
  dP_target << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ddP_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // get Kp and Kd gains
  Eigen::Map<Eigen::Matrix<double, 6, 1>> k_gains_array(k_gains_.data());
  Eigen::Map<Eigen::Matrix<double, 6, 1>> d_gains_array(d_gains_.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> td_gains_array(td_gains_.data());

  Kp = k_gains_array.array().matrix().asDiagonal();
  Kd = d_gains_array.array().matrix().asDiagonal();
  tKd = td_gains_array.array().matrix().asDiagonal();
  
  // initialize clock
  controlller_clock = 0.0;
}

void InverseDynamicsTestController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  // update controller clock
  controlller_clock += period.toSec();

  // get joint angles and angular velocities
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

  // update pinocchio robot model
  pin::forwardKinematics(model, data, q, dq);
  pin::computeJointJacobians(model, data, q); 
  pin::updateFramePlacements(model, data);

  // measure end-effector position and orientation
  p_measured = data.oMf[ee_frame_id].translation();
  R_measured = data.oMf[ee_frame_id].rotation();

  // get end-effector jacobian
  pin::getFrameJacobian(model, data, ee_frame_id, pin::LOCAL_WORLD_ALIGNED, jacobian);

  // get time derivative of positional Jacobian
  pin::getFrameJacobianTimeVariation(model, data, ee_frame_id, pin::LOCAL_WORLD_ALIGNED, djacobian);

  // compute orientation error with targets
  Eigen::Matrix<double, 3, 3> R_error = R_target * R_measured.transpose();
  Eigen::AngleAxisd AngleAxisErr(R_error);
  Eigen::Vector3d rotvec_err = AngleAxisErr.axis() * AngleAxisErr.angle();

  // compute new p_target along the y-axis
  p_target[1] = (1 - std::cos(M_PI * controlller_clock / 5)) * 0.2;

  // compute new dP_target along the y-axis
  dP_target[1] = (M_PI / 5) * std::sin(M_PI * controlller_clock / 5) * 0.2;

  // compute new ddP_cmd along the y-axis
  ddP_cmd[1] = (M_PI * M_PI / 25) * std::cos(M_PI * controlller_clock / 5) * 0.2;

  // compute positional error
  Eigen::Matrix<double, 6, 1> P_error;
  P_error << p_target - p_measured, rotvec_err;

  // compute pseudo-inverse of Jacobian
  pseudoInverse(jacobian, pinv_jacobian);

  // compute joint target
  dP_error = dP_target - jacobian * dq;

  // compute joint torque
  // auto ddq_cmd = pinv_jacobian * (ddP_cmd + p_gain * P_error + d_gain * dP_error - djacobian * dq);
  // auto ddq_cmd = pinv_jacobian * (ddP_cmd + p_gain * P_error + d_gain * dP_error);
  
  // auto ddq_cmd = p_gain * (pinv_jacobian * P_error) + pinv_jacobian * d_gain * dP_target - d_gain * dq; // works
  
  // auto ddq_cmd = pinv_jacobian * (ddP_cmd - djacobian * dq) - Kd * dq + Kp * pinv_jacobian * P_error + Kd * pinv_jacobian * dP_target;
  auto ddq_cmd = pinv_jacobian * (ddP_cmd + Kp * P_error + Kd * dP_target - djacobian * dq) - tKd * dq;
  // auto ddq_cmd = pinv_jacobian * (ddP_cmd + Kp * P_error + Kd * (dP_target - jacobian * dq) - djacobian * dq);

  // get mass matrix
  std::array<double, 49> mass_array = model_handle_->getMass();
  Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass_array.data());

  // get Coriolis and centrifugal terms
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

  torques = M * ddq_cmd + coriolis;

  // Saturate torque rate to avoid discontinuities
  torques << saturateTorqueRate(torques, tau_J_d);

  // set torque
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(torques[i]);
  }

  ROS_INFO_STREAM("Positional Error: " << P_error.transpose());
  // ROS_INFO_STREAM("pJ @ J: \n" << jacobian.transpose() * pinv_jacobian_transpose);
}

Eigen::Matrix<double, 7, 1> InverseDynamicsTestController::saturateTorqueRate(
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

bool InverseDynamicsTestController::read_gains(ros::NodeHandle& node_handle) {
  // check if got p_gain
  if (!node_handle.getParam("p_gain", p_gain)) {
    ROS_ERROR_STREAM("InverseDynamicsTestController: Could not read parameter p_gain");
    return false;
  }

  // check if got d_gain
  if (!node_handle.getParam("d_gain", d_gain)) {
    ROS_ERROR_STREAM("InverseDynamicsTestController: Could not read parameter d_gain");
    return false;
  }

  // check if got dq_gain
  if (!node_handle.getParam("dq_gain", dq_gain)) {
    ROS_ERROR_STREAM("InverseDynamicsTestController: Could not read parameter dq_gain");
    return false;
  }

  return true;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::InverseDynamicsTestController, controller_interface::ControllerBase)