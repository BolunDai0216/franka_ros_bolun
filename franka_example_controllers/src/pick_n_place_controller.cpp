#include <actionlib/client/simple_action_client.h>
#include <controller_interface/controller_base.h>

#include <franka/gripper.h>
#include <franka_example_controllers/pick_n_place_controller.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>

#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include <thread>
#include <cmath>

namespace franka_example_controllers {
double xx, yy, zz;
double x_home, y_home, z_home, cnt1 = 0, cnt2 = 0;

bool PickNPlaceController::init(hardware_interface::RobotHW* robot_hardware,
                               ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();

  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "PickNPlaceController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;

  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("PickNPlaceController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("PickNPlaceController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("PickNPlaceController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "PickNPlaceController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("PickNPlaceController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void PickNPlaceController::starting(const ros::Time& /* time */) {
  // You can initialize any thing here.
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

  elapsed_time_ = ros::Duration(0.0);

  xx = initial_pose_[12];
  yy = initial_pose_[13];
  zz = initial_pose_[14];
}

// Defining the gripper Move action variable (ac).
actionlib::SimpleActionClient<franka_gripper::MoveAction> move_client("franka_gripper/move", true);
actionlib::SimpleActionClient<franka_gripper::GraspAction> grasp_client("franka_gripper/grasp", true);

void PickNPlaceController::update(const ros::Time& /* time */, const ros::Duration& period) {
  std::array<double, 16> new_pose;
  double x, y, z, angle, delta_x, delta_y, delta_z;
  current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

  if (elapsed_time_.toSec() <= 5.0) {
    elapsed_time_ += period;
    x = 0.407464; 
    y = -0.276469;
    z = initial_pose_[14];

    // Calculating the angular angle for gradual motion
    angle = M_PI / 4 * (1 - std::cos((M_PI / 5.0) * elapsed_time_.toSec()));
    
    // Calculating the gradual motion increament for each axis
    delta_x = (x - xx) * std::sin(angle);
    delta_y = (y - yy) * std::sin(angle);
    delta_z = (z - zz) * std::sin(angle);
    
    new_pose = initial_pose_;
    new_pose[12] += delta_x;  // Updating x-axis
    new_pose[13] += delta_y;  // Updating y-axis

    // Sending the new position to robot Arm
    cartesian_pose_handle_->setCommand(new_pose);

    double position_error = std::sqrt(std::pow(x - current_pose_[12], 2.0) + std::pow(y - current_pose_[13], 2.0) + std::pow(z - current_pose_[14], 2.0));
    ROS_INFO_STREAM("PickNPlaceController: Position Error: " << position_error);
  } else if (elapsed_time_.toSec() > 5.0 && elapsed_time_.toSec() <= 6.0) {
    elapsed_time_ += period;

    initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

    xx = initial_pose_[12];
    yy = initial_pose_[13];
    zz = initial_pose_[14];

    // Sending the new position to robot Arm
    cartesian_pose_handle_->setCommand(initial_pose_);
    
  } else if (elapsed_time_.toSec() > 6.0 && elapsed_time_.toSec() <= 11.0) {
    elapsed_time_ += period;
    x = 0.407464; 
    y = -0.276469;
    z = 0.006;

    // Calculating the angular angle for gradual motion
    angle = M_PI / 4 * (1 - std::cos((M_PI / 5.0) * (elapsed_time_.toSec() - 6.0)));
    
    // Calculating the gradual motion increament for each axis
    delta_x = (x - xx) * std::sin(angle);
    delta_y = (y - yy) * std::sin(angle);
    delta_z = (z - zz) * std::sin(angle);
    
    new_pose = initial_pose_;
    new_pose[12] += delta_x;  // Updating x-axis
    new_pose[13] += delta_y;  // Updating y-axis
    new_pose[14] += delta_z;  // Updating z-axis

    // Sending the new position to robot Arm
    cartesian_pose_handle_->setCommand(new_pose);

    double position_error = std::sqrt(std::pow(x - current_pose_[12], 2.0) + std::pow(y - current_pose_[13], 2.0) + std::pow(z - current_pose_[14], 2.0));
    ROS_INFO_STREAM("PickNPlaceController: Position Error: " << position_error);
  } else if (elapsed_time_.toSec() > 11.0 && elapsed_time_.toSec() <= 14.0) {
    elapsed_time_ += period;

    franka_gripper::GraspGoal goal;
    goal.width = 0.0001;
    goal.speed = 0.7;
    goal.force = 10.0;
    goal.epsilon.inner = 0.02;
    goal.epsilon.outer = 0.07;
    grasp_client.sendGoal(goal);

    initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

    xx = initial_pose_[12];
    yy = initial_pose_[13];
    zz = initial_pose_[14];

  } else if (elapsed_time_.toSec() > 14.0 && elapsed_time_.toSec() <= 19.0) {
    elapsed_time_ += period;
    x = 0.407464; 
    y = -0.276469;
    z = 0.479781;

    // Calculating the angular angle for gradual motion
    angle = M_PI / 4 * (1 - std::cos((M_PI / 5.0) * (elapsed_time_.toSec() - 14.0)));
    
    // Calculating the gradual motion increament for each axis
    delta_x = (x - xx) * std::sin(angle);
    delta_y = (y - yy) * std::sin(angle);
    delta_z = (z - zz) * std::sin(angle);
    
    new_pose = initial_pose_;
    new_pose[12] += delta_x;  // Updating x-axis
    new_pose[13] += delta_y;  // Updating y-axis
    new_pose[14] += delta_z;  // Updating z-axis

    // Sending the new position to robot Arm
    cartesian_pose_handle_->setCommand(new_pose); 
  } else if (elapsed_time_.toSec() > 19.0 && elapsed_time_.toSec() <= 20.0) {
    elapsed_time_ += period;

    initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

    xx = initial_pose_[12];
    yy = initial_pose_[13];
    zz = initial_pose_[14];

    // Sending the new position to robot Arm
    cartesian_pose_handle_->setCommand(initial_pose_);
    
  } else if (elapsed_time_.toSec() > 20.0 && elapsed_time_.toSec() <= 25.0) {
    elapsed_time_ += period;
    x = 0.306629;
    y = -0.00131473;

    // Calculating the angular angle for gradual motion
    angle = M_PI / 4 * (1 - std::cos((M_PI / 5.0) * (elapsed_time_.toSec() - 20.0)));
    
    // Calculating the gradual motion increament for each axis
    delta_x = (x - xx) * std::sin(angle);
    delta_y = (y - yy) * std::sin(angle);
    
    new_pose = initial_pose_;
    new_pose[12] += delta_x;  // Updating x-axis
    new_pose[13] += delta_y;  // Updating y-axis

    // Sending the new position to robot Arm
    cartesian_pose_handle_->setCommand(new_pose); 
  } else {
    elapsed_time_ += period;
    cartesian_pose_handle_->setCommand(current_pose_); 
  }

}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::PickNPlaceController,
                       controller_interface::ControllerBase)