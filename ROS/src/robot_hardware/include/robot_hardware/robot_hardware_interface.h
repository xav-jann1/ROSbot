#ifndef ROS_CONTROL__ROBOT_HARDWARE_INTERFACE_H
#define ROS_CONTROL__ROBOT_HARDWARE_INTERFACE_H

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <robot_hardware/robot_hardware.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>

using namespace hardware_interface;

namespace robot_hardware_interface {
static const double POSITION_STEP_FACTOR = 10;
static const double VELOCITY_STEP_FACTOR = 10;

class RobotHardwareInterface : public robot_hardware_interface::RobotHardware {
 public:
  RobotHardwareInterface(ros::NodeHandle& nh);
  ~RobotHardwareInterface();
  void init();
  void update(const ros::TimerEvent& e);
  void read();
  void write(ros::Duration elapsed_time);

  int addJoint(int, std::string, std::string);

 protected:
  // robotcpp::Robot robot;
  ros::NodeHandle nh_;
  ros::Timer non_realtime_loop_;
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  double loop_hz_;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  double p_error_, v_error_, e_error_;
};

}  // namespace robot_hardware_interface

#endif