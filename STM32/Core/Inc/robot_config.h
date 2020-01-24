#ifndef ROBOT_CONFIG_H_
#define ROBOT_CONFIG_H_

#include "stm32f4xx_hal.h"
#include "main.h"

// Loop
#define LOOP_DATA_MS   100
#define LOOP_JOINTS_MS  20

// Topics:
#define JOINTS_DATA_TOPIC    "controller/joints_data"
#define JOINTS_COMMAND_TOPIC "controller/joints_command"

// Encoder:
#define ENCODER_RESOLUTION (1024 * 4 - 1)

// Roues:
#define DIAMETRE_ROUE       0.08485 // m
#define DIAMETRE_ENCODER_L  0.0408 // m
#define DIAMETRE_ENCODER_R  0.0404 // m
#define JOINT_RATIO_L (DIAMETRE_ENCODER_L / DIAMETRE_ROUE)
#define JOINT_RATIO_R (DIAMETRE_ENCODER_R / DIAMETRE_ROUE)

/**
 * Joint wheel_l
 */

// Encoder:
extern TIM_HandleTypeDef htim3;
#define l_enc_def { htim3, ENCODER_RESOLUTION, ENCODER_RELOAD }

// PID:
#define l_pid_values  0.22f, 0.6f, 0.0f
#define l_pid_i_max_min  0.5f, -0.5f
#define l_pid_def { l_pid_values, l_pid_i_max_min }

// Motor:
extern TIM_HandleTypeDef htim1;
#define l_mot_enA { htim1, TIM_CHANNEL_2, PWM_PERIOD, 0.8f, 0.0f }  // PA9
#define l_mot_in1 { Motor_L_IN1_GPIO_Port, Motor_L_IN1_Pin }        // PA7
#define l_mot_in2 { Motor_L_IN2_GPIO_Port, Motor_L_IN2_Pin }        // PA6
#define l_mot_def { l_mot_enA, l_mot_in1, l_mot_in2 }

// Joint:
#define L_JOINT_NAME "wheel_l"
#define l_joint_def { L_JOINT_NAME, JOINT_RATIO_L }

// Création du Joint:
#define wheel_l_defs l_enc_def, l_pid_def, l_mot_def, l_joint_def

/**
 * Joint wheel_r
 */

// Encoder:
extern TIM_HandleTypeDef htim4;
#define r_enc_def { htim4, ENCODER_RESOLUTION, ENCODER_RELOAD }

// PID:
#define r_pid_values  0.22f, 0.6f, 0.0f
#define r_pid_i_max_min  0.5f, -0.5f
#define r_pid_def { r_pid_values, r_pid_i_max_min }

// Motor:
extern TIM_HandleTypeDef htim1;
#define r_mot_enA { htim1, TIM_CHANNEL_1, PWM_PERIOD, 0.8f, 0.0f }  // PA8
#define r_mot_in1 { Motor_R_IN1_GPIO_Port, Motor_R_IN1_Pin }        // PB9
#define r_mot_in2 { Motor_R_IN2_GPIO_Port, Motor_R_IN2_Pin }        // PB8
#define r_mot_def { r_mot_enA, r_mot_in1, r_mot_in2 }

// Joint:
#define R_JOINT_NAME "wheel_r"
#define r_joint_def { R_JOINT_NAME, JOINT_RATIO_R }

// Création du Joint:
#define wheel_r_defs r_enc_def, r_pid_def, r_mot_def, r_joint_def

#endif /* ROBOT_CONFIG_H_ */
