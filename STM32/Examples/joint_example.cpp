#include "main.h"
#include "mainpp.h"
#include "robot_VelocityJoint.h"
#include "robot_node.h"
#include "robot_joint.h"
#include "robot_config_joint_example.h"
#include "ros.h"

// VelocityJoint:
robot::VelocityJoint joint1(joint1_defs);

// Node:
ros::NodeHandle nh;

// Setup node:
void setup(void) {
  // Joint:
  robot::initJoint(joint1, nh);

  // Node:
  robot::initNode(nh);
  nh.loginfo("STM32 Connecté !");

  // Params server:
  robot::getJointPidValues(joint1, "/robot/pid", nh);

  nh.loginfo("> Fin de l'initialisation");
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

// Loop:
void loop() {
  joint1.update();

  // Publie des données:
  static int t = 0;
  if (HAL_GetTick() - t >= LOOP_DATA_MS ) {
    t = HAL_GetTick();
    joint1.publishEncoderData();
    joint1.publishPidValues();
    joint1.publishData();
  }

  // Joints data:
  static int t2 = 0;
  if (HAL_GetTick() - t2 >= LOOP_JOINTS_MS ) {
    robot::publishJointsData();
    t2 = HAL_GetTick();
  }

  nh.spinOnce();
  HAL_Delay(LOOP_JOINTS_MS);
}
