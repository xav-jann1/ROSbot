#include "mainpp.h"
#include "main.h"
#include "robot/config.h"
#include "robot/VelocityJoint.h"
#include <stdio.h>

// Motors: (pour commander en PWM)
robot::Motor motor_l(l_mot_def);
robot::Motor motor_r(r_mot_def);

// Encoders: (pour récupérer les données)
robot::Encoder encoder_l(l_enc_def);
robot::Encoder encoder_r(r_enc_def);

// VelocityJoint:
// (pour ne pas avoir d'erreur lors de la compilation à cause de robot_node.cpp)
robot::VelocityJoint wheel_l_joint(wheel_l_defs);
robot::VelocityJoint wheel_r_joint(wheel_r_defs);

// Loop delays:
int t = LOOP_JOINTS_MS + 1;

// Position et vitesse des encodeurs:
float pos_l, vel_l, pos_r, vel_r;

void setup() {
  // Initialisations:
  motor_l.init();
  motor_r.init();
  encoder_l.init();
  encoder_r.init();

  // Pointeurs:
  printf("Pointeurs:\r\n");
  printf("- pos_l: %p\r\n", &pos_l);
  printf("- vel_l: %p\r\n", &vel_l);
  printf("- pos_r: %p\r\n", &pos_r);
  printf("- vel_r: %p\r\n", &vel_r);

  // Démarrage:
  t = 0;
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  HAL_Delay(1000);
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  HAL_Delay(1000);
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

  // Avance pendant 2 secondes:
  float pwm = 0.5;
  motor_l.forward();
  motor_r.forward();
  motor_l.pwm(pwm);
  motor_r.pwm(pwm);
  HAL_Delay(2000);

  // Arrêt:
  motor_l.off();
  motor_r.off();

  // Eteint LED:
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

void loop() {

}

// Loop 1 ms:
float LOOP_JOINTS_S = LOOP_JOINTS_MS / 1000.0f;
void HAL_SYSTICK_Callback() {

  // Mise à jour vitesse et position des encodeurs:
  pos_l = encoder_l.getPosition();
  vel_l = encoder_l.getVelocity();
  pos_r = encoder_r.getPosition();
  vel_r = encoder_r.getVelocity();

  // Mise à jour des joints:
  if (++t == LOOP_JOINTS_MS) {
    t = 0;
    encoder_l.update(LOOP_JOINTS_S);
    encoder_r.update(LOOP_JOINTS_S);
  }

}
