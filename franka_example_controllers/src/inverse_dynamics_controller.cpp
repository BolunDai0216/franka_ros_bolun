#include <franka_example_controllers/inverse_dynamics_controller.h>
#include <franka/robot_state.h>
#include <franka_example_controllers/pseudo_inversion.h>

#include <cmath>

#include <controller_interface/controller_base.h> 
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>   

namespace pin = pinocchio;

namespace franka_example_controllers {

bool InverseDynamicsController::init(hardware_interface::RobotHW* robot_hw,
                                          ros::NodeHandle& node_handle) {
  // check if got arm_id
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("InverseDynamicsController: Could not read parameter arm_id");
    return false;
  }
  
  // check if got joint_names
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("InverseDynamicsController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("InverseDynamicsController: Wrong number of joint names, got " << joint_names.size() << " instead of 7 names!");
    return false;
  }
  
  // check if model_handle_ works
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "InverseDynamicsController: Error getting model interface from hardware");
    return false;
  }

  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "InverseDynamicsController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  // check if state_handle_ works
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "InverseDynamicsController: Error getting state interface from hardware");
    return false;
  }
  
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "InverseDynamicsController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  // check if joint_handles_ got each joint (crucial step of avoiding could not switch controller error)
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "InverseDynamicsController: Error getting effort joint interface from hardware");
    return false;
  }
  
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "InverseDynamicsController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // build pin_robot from urdf
  std::string urdf_filename = "/home/parallels/bolun_ws/src/franka_ros_bolun/franka_example_controllers/fr3.urdf";
  pin::urdf::buildModel(urdf_filename, model);
  data = pin::Data(model);

  return true;
}

void InverseDynamicsController::starting(const ros::Time& /* time */) {
  // get intial robot state
  franka::RobotState initial_state = state_handle_->getRobotState();

  // set movement duration
  movement_duration = 10.0;

  // initialize controller clock
  controlller_clock = 0.0;

  // get initial end-effector position and orientation
  Eigen::Affine3d T_EE(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  p_start = T_EE.translation();
  R_start = T_EE.rotation();
  
  // set terminal end-effector position and orientation
  p_end << 0.3, 0.3, 0.2;
  R_end = T_EE.rotation();

  // compute orientation error between initial and terminal configuration
  R_error = R_end * R_start.transpose();
  Eigen::AngleAxisd AngleAxisError(R_error);
  orientation_error_axis = AngleAxisError.axis();
  orientation_error_angle = AngleAxisError.angle();

  ee_frame_id = model.getFrameId("fr3_hand_tcp");
}

void InverseDynamicsController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  // update controller clock
  controlller_clock += period.toSec();

  // get joint angles and angular velocities
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  // get current end-effector position and orientation
  Eigen::Affine3d T_EE_current(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  p_current = T_EE_current.translation();
  R_current = T_EE_current.rotation();

  // update pinocchio robot model
  pin::forwardKinematics(model, data, q, dq);
  pin::computeJointJacobians(model, data, q); 
  pin::updateFramePlacements(model, data); 

  // get Jacobian matrix
  pin::getFrameJacobian(model, data, ee_frame_id, pin::LOCAL_WORLD_ALIGNED, J);

  // get time derivative of positional Jacobian
  pin::computeJointJacobiansTimeVariation(model, data, q, dq);
  pin::getFrameJacobianTimeVariation(model, data, ee_frame_id, pin::LOCAL_WORLD_ALIGNED, dJ);

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
  Eigen::Matrix<double, 6, 1> P_err; 
  Eigen::Matrix<double, 6, 1> dP_target;
  Eigen::Matrix<double, 6, 1> ddP_target;

  P_err << p_error, rotvec_err;
  dP_target << v_target, w_target;
  ddP_target << a_target, dw_target; 

  // compute pseudo-inverse of Jacobian
  Eigen::MatrixXd pJ_EE;
  pseudoInverse(J, pJ_EE);

  // get estimated dP
  auto dP = J * dq;
  auto a = ddP_target + 100 * P_err + 10 * (dP_target - dP) - dJ * dq;
  auto ddq_desired = pJ_EE * a;

  // get mass matrix
  std::array<double, 49> mass_array = model_handle_->getMass();
  Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass_array.data());

  // get Coriolis and centrifugal terms
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

  torques = M * (ddq_desired - 10 * dq) + coriolis;
  // torques = 300 * (pJ_EE * P_err) + 10 * (pJ_EE * dP_target - dq);

  // set torque
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(torques[i]);
  }

  ROS_INFO_STREAM("ee_pos: " << p_current);
}

void InverseDynamicsController::alpha_func(const double& t) {
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

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::InverseDynamicsController, controller_interface::ControllerBase)