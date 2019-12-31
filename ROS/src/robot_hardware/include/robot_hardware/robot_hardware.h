#ifndef ROS_CONTROL__ROBOT_HARDWARE_H
#define ROS_CONTROL__ROBOT_HARDWARE_H

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>

namespace robot_hardware_interface {
/// \brief Hardware interface for a robot
class RobotHardware : public hardware_interface::RobotHW {
 protected:
  // Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;

  // Custom or available transmissions
  // transmission_interface::RRBOTTransmission rrbot_trans_;
  // std::vector<transmission_interface::SimpleTransmission> simple_trans_;

  // Shared memory
  int num_joints_;
  int joint_mode_;  // position, velocity, or effort
  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;

};  // class

}  // namespace robot_hardware_interface

#endif