#include "mainpp.h"
#include "main.h"
#include <stdio.h>

#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "robot/VelocityJoint.h"

// Constantes déjà configurées dans 'Device Configuration Tool':
#define ENCODER_PERIOD 0xFFFF
#define ENCODER_RELOAD (ENCODER_PERIOD / 2)
#define ENCODER_RESOLUTION 64
#define ENCODER_LOOP_MS 20

// Encoder:
extern TIM_HandleTypeDef htim3;
#define enc_def { htim3, ENCODER_RESOLUTION, ENCODER_RELOAD }

// PID:
float pid_p = 0.5f, pid_i = 0.2f, pid_d = 0.0f;
float pid_i1 = 0.3f, pid_i2 = -0.3f;
#define pid_def { pid_p, pid_i, pid_d, pid_i1, pid_i2 }

// Motor:
extern TIM_HandleTypeDef htim2;
#define mot_enA { htim2, TIM_CHANNEL_3, PWM_PERIOD, 0.8f, 0.2f }
#define mot_in1 { GPIOB, GPIO_PIN_4 }
#define mot_in2 { GPIOB, GPIO_PIN_5 }
#define mot_def { mot_enA, mot_in1, mot_in2 }

// Joint:
#define JOINT_NAME "joint"
#define JOINT_RATIO 1/43.8f
#define joint_def { JOINT_NAME, JOINT_RATIO }
#define LOOP_DATA_MS 100

// VelocityJoint:
robot::VelocityJoint joint(enc_def, pid_def, mot_def, joint_def);

// Node:
ros::NodeHandle nh;

// Subscriber:
void cmd_cb(const std_msgs::Float32 &msg) {
  joint.command(msg.data);
}
ros::Subscriber<std_msgs::Float32> cmd_sub("set_cmd", &cmd_cb);

// Setup node:
void setup(void) {
  // Node:
  nh.initNode();

  // Joint:
  joint.init();
  joint.command(2);

  // Publishers, Subscribers:
  joint.addPidSubscribers(nh);
  joint.addEncoderPublishers(nh);
  nh.subscribe(cmd_sub);
  nh.spinOnce();
}

// Loop:
float dt = ENCODER_LOOP_MS / 1000.0f;
void loop() {
  joint.update();

  static int t = 0;
  if (HAL_GetTick() - t >= LOOP_DATA_MS ) {
    // Publie les données de l'encodeur:
    joint.publishEncoderData();
    t = HAL_GetTick();
  }

  nh.spinOnce();
  HAL_Delay(ENCODER_LOOP_MS);
}
