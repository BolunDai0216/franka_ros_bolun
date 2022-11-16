#include <franka_example_controllers/proxsuite_controller.h>
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

bool ProxsuiteController::init(hardware_interface::RobotHW* robot_hw,
                                          ros::NodeHandle& node_handle) {
  // check if got arm_id
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("ProxsuiteController: Could not read parameter arm_id");
    return false;
  }
  
  // check if got joint_names
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("ProxsuiteController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("ProxsuiteController: Wrong number of joint names, got " << joint_names.size() << " instead of 7 names!");
    return false;
  }
  
  // check if model_handle_ works
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "ProxsuiteController: Error getting model interface from hardware");
    return false;
  }

  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ProxsuiteController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  // check if state_handle_ works
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "ProxsuiteController: Error getting state interface from hardware");
    return false;
  }
  
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ProxsuiteController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  // check if joint_handles_ got each joint (crucial step of avoiding could not switch controller error)
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "ProxsuiteController: Error getting effort joint interface from hardware");
    return false;
  }
  
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "ProxsuiteController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  return true;
}

void ProxsuiteController::starting(const ros::Time& /* time */) {
  // get intial robot state
  franka::RobotState initial_state = state_handle_->getRobotState();

  // get joint angles and angular velocities
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(initial_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(initial_state.dq.data());

  // get current end-effector position and orientation
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  p_start = transform.translation();
  R_start = transform.rotation();

  // set movement duration
  movement_duration = 10.0;

  // initialize controller clock
  controlller_clock = 0.0;
  
  // set terminal end-effector position and orientation
  p_end << 0.4, 0.4, 0.2;
  R_end = transform.rotation();

  // compute orientation error between initial and terminal configuration
  R_error = R_end * R_start.transpose();
  Eigen::AngleAxisd AngleAxisError(R_error);
  orientation_error_axis = AngleAxisError.axis();
  orientation_error_angle = AngleAxisError.angle();

  Kp << 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 10.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 10.0;

  Kd << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  // initialize QP parameters
  qp_H = Eigen::MatrixXd::Zero(14, 14);
  qp_g = Eigen::MatrixXd::Zero(14, 1);

  q_nominal << q[0], q[1], q[2], q[3], q[4], q[5], q[6];
}

void ProxsuiteController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  // update controller clock
  controlller_clock += period.toSec();

  // get joint angles and angular velocities
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  // get end-effector jacobian
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

  // get current end-effector position and orientation
  Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  p_current = current_transform.translation();
  R_current = current_transform.rotation();

  // get α, dα, ddα
  alpha_func(controlller_clock);

  // compute positional targets
  p_target = p_start + alpha * (p_end - p_start);
  v_target = dalpha * (p_end - p_start);
  a_target = ddalpha * (p_end - p_start);

  // compute orientaional targets
  angle_target = alpha * orientation_error_angle;
  Eigen::AngleAxisd AngleAxisTarget(angle_target, orientation_error_axis);
  R_target = AngleAxisTarget.toRotationMatrix() * R_start;
  w_target = dalpha * orientation_error_angle * orientation_error_axis;
  dw_target = ddalpha * orientation_error_angle * orientation_error_axis;

  // compute orientation error with targets
  Eigen::Matrix<double, 3, 3> R_err = R_target * R_current.transpose();
  Eigen::AngleAxisd AngleAxisErr(R_err);
  Eigen::Vector3d rotvec_err = AngleAxisErr.axis() * AngleAxisErr.angle();

  // compute position error with targets
  Eigen::Matrix<double, 3, 1> p_error = p_target - p_current;

  // get errors and targets for torque computation
  P_err << p_error, rotvec_err;

  // compute pseudo-inverse of Jacobian
  pseudoInverse(J, pJ_EE);

  get_qp_parameters(q, dq);
  solve_qp();

  torques = 10 * (q_desired - q) - 0.1 * dq;

  // Saturate torque rate to avoid discontinuities
  torques << saturateTorqueRate(torques, tau_J_d);

  // set torque
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(torques[i]);
  }

  ROS_INFO_STREAM("ee_pos: " << p_current);

  if (controlller_clock >= (movement_duration + 2.0)) {
    controlller_clock = 0.0;

    // get current end-effector position and orientation
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    p_start = transform.translation();
    R_start = transform.rotation();

    // set terminal end-effector position and orientation
    p_end << 0.4, -p_end[1], 0.2;
    R_end = transform.rotation();

    // compute orientation error between initial and terminal configuration
    R_error = R_end * R_start.transpose();
    Eigen::AngleAxisd AngleAxisError(R_error);
    orientation_error_axis = AngleAxisError.axis();
    orientation_error_angle = AngleAxisError.angle();
  }
}

Eigen::Matrix<double, 7, 1> ProxsuiteController::saturateTorqueRate(
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

void ProxsuiteController::alpha_func(const double& t) {
  if (t <= movement_duration){
    double sin_ = std::sin(M_PI * t / movement_duration);
    double cos_ = std::cos(M_PI * t / movement_duration);
    double beta = (M_PI / 4) * (1 - cos_);
    double _sin = std::sin(beta);
    double _cos = std::cos(beta);

    double T2 = movement_duration * movement_duration;

    alpha = _sin;
    dalpha = (M_PI * M_PI / (4 * movement_duration)) * _cos * sin_;
    ddalpha = (std::pow(M_PI, 3) / (4 * T2)) * cos_ * _cos - (std::pow(M_PI, 4) / (16 * T2)) * sin_ * sin_ * _sin;
  } else {
    alpha = 1.0;
    dalpha = 0.0;
    ddalpha = 0.0;
  }
}

void ProxsuiteController::solve_qp(void) {
  proxsuite::proxqp::isize dim = 7;
  proxsuite::proxqp::isize n_eq = 0;
  proxsuite::proxqp::isize n_in = 0;
  proxsuite::proxqp::dense::QP<double> qp(dim, n_eq, n_in); // create the QP object

  qp.init(qp_H, qp_g, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt);
  qp.solve();
  q_desired << qp.results.x[0], 
               qp.results.x[1], 
               qp.results.x[2], 
               qp.results.x[3],
               qp.results.x[4],
               qp.results.x[5],
               qp.results.x[6];
}

void ProxsuiteController::get_qp_parameters(const Eigen::Matrix<double, 7, 1>& q, 
                                            const Eigen::Matrix<double, 7, 1>& dq) {
  auto Pr = Eigen::MatrixXd::Identity(7, 7) - pJ_EE * J;

  qp_H = 2 * J.transpose() * J + 2 * Pr.transpose() * Pr;
  qp_g = -2 * (q.transpose() * J.transpose() * J + P_err.transpose() * J + q_nominal.transpose() * Pr.transpose() * Pr).transpose();
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ProxsuiteController, controller_interface::ControllerBase)