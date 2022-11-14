#include <franka_example_controllers/inverse_dynamics_controller.h>

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
  std::string urdf_filename = "/home/bolun/bolun_ws/src/franka_ros_bolun/franka_example_controllers/fr3.urdf";
  pin::urdf::buildModel(urdf_filename, model);
  data = pin::Data(model);

  return true;
}

void InverseDynamicsController::starting(const ros::Time& /* time */) {
  // get intial robot state
  franka::RobotState initial_state = state_handle_->getRobotState();

  // set movement duration
  movement_duration = 30.0;

  // initialize controller clock
  controlller_clock = 0.0;
}

void InverseDynamicsController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  // update controller clock
  controlller_clock += period.toSec();

  // get joint angles and angular velocities
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  // update pinocchio robot model
  pin::forwardKinematics(model, data, q, dq);
  pin::updateFramePlacements(model, data); 

  alpha_func(controlller_clock);

  // set torque
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(0.0 - dq[i]);
  }
}

void InverseDynamicsController::alpha_func(const double& t) {
  if (t < movement_duration){
    double beta = (M_PI / 4) * (1 - std::cos(M_PI * t / movement_duration));
    double _sin = std::sin(beta);
    double _cos = std::cos(beta);
    double sin_ = std::sin(M_PI * t / movement_duration);
    double cos_ = std::cos(M_PI * t / movement_duration);
    double T2 = movement_duration * movement_duration;
    double sin__ = sin_ * sin_;

    alpha = _sin;
    dalpha = (M_PI * M_PI / (4 * movement_duration)) * _cos * sin_;
    ddalpha = (std::pow(M_PI, 3) / (4 * T2)) * cos_ * _cos - (std::pow(M_PI, 4) / (16 * T2)) * sin__ * _sin;
  } else {
    alpha = 1.0;
    dalpha = 0.0;
    ddalpha = 0.0;
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::InverseDynamicsController, controller_interface::ControllerBase)