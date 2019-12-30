#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <robot_hardware/robot_hardware_interface.h>
// #include <robotcpp/robot.h>
#include <sstream>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

namespace robot_hardware_interface {
RobotHardwareInterface::RobotHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
  init();
  controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
  nh_.param("/robot/hardware_interface/loop_hz", loop_hz_, 0.1);
  ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);
  non_realtime_loop_ = nh_.createTimer(update_freq, &RobotHardwareInterface::update, this);
}

RobotHardwareInterface::~RobotHardwareInterface() {}

void RobotHardwareInterface::init() {
  // Get joint names
  nh_.getParam("/robot/hardware_interface/joints", joint_names_);
  num_joints_ = joint_names_.size();

  // Resize vectors
  joint_position_.resize(num_joints_);
  joint_velocity_.resize(num_joints_);
  joint_effort_.resize(num_joints_);
  joint_position_command_.resize(num_joints_);
  joint_velocity_command_.resize(num_joints_);
  joint_effort_command_.resize(num_joints_);

  // Initialize Controller
  /*for (int i = 0; i < num_joints_; ++i) {
    robotcpp::Joint joint = robot.getJoint(joint_names_[i]);

    // Create joint state interface
    JointStateHandle jointStateHandle(joint.name, &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
    joint_state_interface_.registerHandle(jointStateHandle);

    // Create position joint interface
    JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
    JointLimits limits;
    SoftJointLimits softLimits;
    getJointLimits(joint.name, nh_, limits);
    PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
    positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
    position_joint_interface_.registerHandle(jointPositionHandle);

    // Create effort joint interface
    JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
    effort_joint_interface_.registerHandle(jointEffortHandle);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
  registerInterface(&effort_joint_interface_);
  registerInterface(&positionJointSoftLimitsInterface);
  */

  /*** Code adapté du tutoriel de ros_control: "Create your own hardware interface" ***/

  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_a("wheel_l", &pos[0], &vel[0], &eff[0]);
  joint_state_interface_.registerHandle(state_handle_a);

  hardware_interface::JointStateHandle state_handle_b("wheel_r", &pos[1], &vel[1], &eff[1]);
  joint_state_interface_.registerHandle(state_handle_b);

  registerInterface(&joint_state_interface_);

  // connect and register the joint position interface
  hardware_interface::JointHandle pos_handle_a(joint_state_interface_.getHandle("wheel_l"), &cmd[0]);
  velocity_joint_interface_.registerHandle(pos_handle_a);

  hardware_interface::JointHandle pos_handle_b(joint_state_interface_.getHandle("wheel_r"), &cmd[1]);
  velocity_joint_interface_.registerHandle(pos_handle_b);

  registerInterface(&velocity_joint_interface_);
}

void RobotHardwareInterface::update(const ros::TimerEvent& e) {
  elapsed_time_ = ros::Duration(e.current_real - e.last_real);
  read();
  controller_manager_->update(ros::Time::now(), elapsed_time_);
  write(elapsed_time_);
}

void RobotHardwareInterface::read() {
  /*for (int i = 0; i < num_joints_; i++) {
    joint_position_[i] = robot.getJoint(joint_names_[i]).read();
  }*/
}

void RobotHardwareInterface::write(ros::Duration elapsed_time) {
  /*positionJointSoftLimitsInterface.enforceLimits(elapsed_time);
  for (int i = 0; i < num_joints_; i++) {
    robot.getJoint(joint_names_[i]).actuate(joint_effort_command_[i]);
  }*/

  // Boucle fermée directe:
  double v1 = cmd[0];
  double v2 = cmd[1];

  pos[0] += v1;
  pos[1] += v2;

  vel[0] = v1;
  vel[1] = v2;

  ROS_INFO("cmd: %.2f, %.2f", v1, v2);
}
}  // namespace robot_hardware_interface