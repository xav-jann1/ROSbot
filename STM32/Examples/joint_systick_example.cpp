#include "main.h"
#include "mainpp.h"
#include "robot/node/VelocityJointWithTopics.h"
#include "robot/node/node.h"
#include "robot/node/joint.h"
#include "robot/config_joint_example.h"
#include "ros.h"

// VelocityJoint:
robot::VelocityJointWithTopics joint1(joint1_defs);

// Node:
ros::NodeHandle nh;

// Loop delays:
int t = LOOP_JOINTS_MS + 1, t1 = LOOP_DATA_MS + 1;

// Setup node:
void setup(void) {
  // Joint:
  robot::initJoint(joint1, nh);

  // Node:
  robot::initNode(nh);
  nh.loginfo("STM32 Connecté !");

  // Parameters server:
  robot::getJointPidValues(joint1, "/robot/pid", nh);

  nh.loginfo("> Fin de l'initialisation");
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  t = t1 = 0;
}

// Loop:
void loop() {
  nh.spinOnce();
}

// Loop 1 ms:
void HAL_SYSTICK_Callback() {
  // Mise à jour des joints:
  if (++t == LOOP_JOINTS_MS) {
    t = 0;
    joint1.update();
    robot::publishJointsData();
  }

  // Publie des données:
  if (++t1 == LOOP_DATA_MS) {
    t1 = 0;
    joint1.publishEncoderData();
    joint1.publishPidValues();
    joint1.publishData();
  }
}
