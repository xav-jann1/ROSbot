#include "main.h"
#include "mainpp.h"
#include "robot/VelocityJoint.h"
#include "robot/node/node.h"
#include "robot/node/joint.h"
#include "robot/config.h"
#include "utils.h"
#include "ros.h"

// VelocityJoint:
robot::VelocityJointWithTopics wheel_l_joint(wheel_l_defs);
robot::VelocityJointWithTopics wheel_r_joint(wheel_r_defs);

// Node:
ros::NodeHandle nh;

// Loop delays:
int t = LOOP_JOINTS_MS + 1, t1 = LOOP_DATA_MS + 1;

// Setup node:
void setup(void) {
  // Joint:
  robot::initJoint(wheel_l_joint, nh);
  robot::initJoint(wheel_r_joint, nh);

  // Node:
  robot::initNode(nh);
  nh.loginfo("STM32 Connecté !");

  // Active les envois de données de la boucle 1 ms:
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
    wheel_l_joint.update();
    wheel_r_joint.update();
    robot::publishJointsData();
  }

  // Publie des données:
  if (++t1 == LOOP_DATA_MS) {
    t1 = 0;

    wheel_l_joint.publishData();
    wheel_r_joint.publishData();

    //wheel_l_joint.publishEncoderData();
    //wheel_r_joint.publishEncoderData();

    //wheel_l_joint.publishPidValues();
    //wheel_r_joint.publishPidValues();
  }

  // Mise à jour de la connexion avec ROS:
  robot::updateNode(nh);
}
