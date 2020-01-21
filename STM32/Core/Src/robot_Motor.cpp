#include "robot_Motor.h"

namespace robot {

Motor::Motor(MotorDef mot_def) :
    pin_pwm_(mot_def.en), pin_in1_(mot_def.in1), pin_in2_(mot_def.in2), isOn_(true) {
}

void Motor::init() {
  if (HAL_TIM_PWM_Start(&pin_pwm_.htim, pin_pwm_.channel) != HAL_OK) {
    Init_Error_Handler();
  }
  pwm(0);
}

void Motor::pwm(float pwm) { // pwm: [0, 1]
  if (!isOn_) return;
  pwm = abs(pwm) > 1 ? 1 : abs(pwm);
  uint32_t pulse_width = pin_pwm_.period * pwm;
  __HAL_TIM_SET_COMPARE(&pin_pwm_.htim, pin_pwm_.channel, pulse_width);
}

/*** Déplacement ***/

void Motor::forward() {
  HAL_GPIO_WritePin(pin_in1_.port, pin_in1_.pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(pin_in2_.port, pin_in2_.pin, GPIO_PIN_RESET);
}

void Motor::backward() {
  HAL_GPIO_WritePin(pin_in1_.port, pin_in1_.pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(pin_in2_.port, pin_in2_.pin, GPIO_PIN_SET);
}

void Motor::stop() {
  HAL_GPIO_WritePin(pin_in1_.port, pin_in1_.pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(pin_in2_.port, pin_in2_.pin, GPIO_PIN_RESET);
  pwm(0.0f);
}

void Motor::off() {
  stop();
  isOn_ = false;
}

void Motor::on() {
  isOn_ = true;
}

// Avance ou recule en fonction du pwm:
void Motor::turn(float pwm) {
  if (pwm < 0) {
    pwm -= pin_pwm_.offset;
    if (pwm < -pin_pwm_.max)
      pwm = -pin_pwm_.max;
    this->pwm(-pwm);
    backward();
  } else {
    pwm += pin_pwm_.offset;
    if (pwm > pin_pwm_.max)
      pwm = pin_pwm_.max;
    this->pwm(pwm);
    forward();
  }
}

} /* namespace robot */
