#ifndef ROS_CONTROL__ROBOT_HARDWARE_INTERFACE_H
#define ROS_CONTROL__ROBOT_HARDWARE_INTERFACE_H

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <robot_hardware/robot_hardware.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/SetBool.h>
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
  void joint_command_publish(int, float);
  void joints_data_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  bool write_joints_srv(std_srvs::SetBool::Request&, std_srvs::SetBool::Response&);

 protected:
  ros::NodeHandle nh_;
  ros::Timer non_realtime_loop_;
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  double loop_hz_;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  double p_error_, v_error_, e_error_;

  // Publishers:
  ros::Publisher joints_command_publisher_;
  std::vector<ros::Publisher> joint_command_publisher_;

  // Subscriber:
  ros::Subscriber joints_data_sub_;

  // Autre:
  bool simulate_joints_;
  bool write_joints_;
  ros::ServiceServer write_joints_service_;
};

}  // namespace robot_hardware_interface

#endif