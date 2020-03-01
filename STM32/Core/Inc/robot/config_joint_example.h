#ifndef ROBOT_CONFIG_H_
#define ROBOT_CONFIG_H_

#include "stm32f4xx_hal.h"

// Loop
#define LOOP_DATA_MS   100
#define LOOP_JOINTS_MS  20

// Topics:
#define JOINTS_DATA_TOPIC    "controller/joints_data2"
#define JOINTS_COMMAND_TOPIC "controller/joints_command"

// Encoder:
// Constantes déjà configurées dans 'Device Configuration Tool':
#define ENCODER_PERIOD 0xFFFF
#define ENCODER_RELOAD (ENCODER_PERIOD / 2)
#define ENCODER_RESOLUTION 64

/***
 * Joint 1
 */

// Encoder:
extern TIM_HandleTypeDef htim3;
#define enc_def { htim3, ENCODER_RESOLUTION, ENCODER_RELOAD }

// PID:
#define pid_values  0.5f, 0.2f, 0.0f
#define pid_i_max_min  0.3f, -0.3f
#define joint1_pid_def { pid_values, pid_i_max_min }

// Motor:
extern TIM_HandleTypeDef htim2;
#define mot_enA { htim2, TIM_CHANNEL_3, PWM_PERIOD, 0.8f, 0.2f }
#define mot_in1 { GPIOB, GPIO_PIN_4 }
#define mot_in2 { GPIOB, GPIO_PIN_5 }
#define mot_def { mot_enA, mot_in1, mot_in2 }

// Joint:
#define JOINT_NAME "wheel_right"
#define JOINT_RATIO 1/43.8f
#define joint_def { JOINT_NAME, JOINT_RATIO }

// Création du Joint:
#define joint1_defs enc_def, joint1_pid_def, mot_def, joint_def

#endif /* INC_ROBOT_CONFIG_H_ */
