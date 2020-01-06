#include <robot_hardware/robot_hardware_interface.h>
int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_hardware_interface");
  ros::CallbackQueue ros_queue;

  ros::NodeHandle nh;
  nh.setCallbackQueue(&ros_queue);
  robot_hardware_interface::RobotHardwareInterface rhi(nh);

  ros::MultiThreadedSpinner spinner(0);
  spinner.spin(&ros_queue);
  return 0;
}