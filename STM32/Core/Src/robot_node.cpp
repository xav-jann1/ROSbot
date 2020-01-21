#include "robot_node.h"

// Joints:
extern robot::VelocityJoint wheel_l_joint;
extern robot::VelocityJoint wheel_r_joint;

namespace robot {

// Publisher:
std_msgs::Float64MultiArray joints_data_msg;
ros::Publisher joints_data_pub(JOINTS_DATA_TOPIC, &joints_data_msg);

// Subscriber:
void joints_command_cb(const std_msgs::Float64MultiArray &msg) {
  wheel_l_joint.command(msg.data[0]);
  wheel_r_joint.command(msg.data[1]);

  // Clignote la LED lorsque des messages sont reçus:
  static int counter = 0;
  if (++counter == 10) {
    counter = 0;
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  }
}
ros::Subscriber<std_msgs::Float64MultiArray> joints_command_sub(JOINTS_COMMAND_TOPIC, &joints_command_cb);


void initNode(ros::NodeHandle& nh) {
  // Prépare le message:
  initJointsDataMsg();

  // Joints publisher, subscriber:
  nh.advertise(joints_data_pub);
  nh.subscribe(joints_command_sub);

  // Node:
  nh.initNode();

  // Attend la connexion avec ROS:
  while(!nh.connected()) nh.spinOnce();
}

void updateNode(ros::NodeHandle& nh) {
  // S'il n'y a plus de connexion:
  if (!nh.connected()) {
    // Eteint les moteurs et la LED:
    wheel_l_joint.command(0.0f);
    wheel_r_joint.command(0.0f);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  }
}

void publishJointsData() {
  // Remplie le message:
  joints_data_msg.data[0] = wheel_l_joint.getPosition();
  joints_data_msg.data[1] = wheel_l_joint.getVelocity();
  joints_data_msg.data[2] = wheel_l_joint.getPWM();
  joints_data_msg.data[3] = wheel_r_joint.getPosition();
  joints_data_msg.data[4] = wheel_r_joint.getVelocity();
  joints_data_msg.data[5] = wheel_l_joint.getPWM();

  // Publie le message:
  joints_data_pub.publish(&joints_data_msg);
}

void initJointsDataMsg() {
  joints_data_msg.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  joints_data_msg.layout.dim[0].label = "i";
  joints_data_msg.layout.dim[0].size = 6;
  joints_data_msg.layout.dim[0].stride = 1*6;
  joints_data_msg.data = (float *)malloc(sizeof(float)*8);
  joints_data_msg.data_length = 6;
}

}
