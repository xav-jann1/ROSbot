#include "mainpp.h"
#include "main.h"
#include <stdio.h>

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
extern TIM_HandleTypeDef htim1;
#define mot_enA { htim1, TIM_CHANNEL_1, PWM_PERIOD, 0.8f, 0.2f }
#define mot_in1 { GPIOB, GPIO_PIN_9 }
#define mot_in2 { GPIOB, GPIO_PIN_8 }
#define mot_def { mot_enA, mot_in1, mot_in2 }

// Joint:
#define JOINT_NAME "joint"
#define JOINT_RATIO 1/43.8f
#define joint_def { JOINT_NAME, JOINT_RATIO }
#define LOOP_DATA_MS 100

// VelocityJoint:
robot::VelocityJoint joint(enc_def, pid_def, mot_def, joint_def);

// Setup node:
void setup(void) {
  joint.init();
  joint.command(2);
}

// Loop:
float dt = ENCODER_LOOP_MS / 1000.0f;
void loop() {
  float pwm = joint.update();

  static int t = 0;
  if (HAL_GetTick() - t >= LOOP_DATA_MS ) {
    printf("pos: %.2f, vel: %.2f, pwm: %.2f\r\n", joint.getPosition(), joint.getVelocity() * JOINT_RATIO, pwm);
    t = HAL_GetTick();
  }

  HAL_Delay(ENCODER_LOOP_MS);
}
