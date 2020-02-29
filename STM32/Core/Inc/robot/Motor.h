#ifndef ROBOT_MOTOR_H_
#define ROBOT_MOTOR_H_

#include "utils.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>  /* abs */

namespace robot {

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
} Pin;

typedef struct {
  TIM_HandleTypeDef &htim;
  uint32_t channel;
  uint32_t period;
  float max;
  float offset;
} PwmPin;

typedef struct {
  PwmPin en;
  Pin in1;
  Pin in2;
} MotorDef;

class Motor {
public:
  Motor(MotorDef);

  void init();

  // Déplacement:
  void forward();
  void backward();
  void stop();
  void turn(float);
  void pwm(float);

  // Setters:
  void setPWM_max(float pwm) { // pwm: [0, 1]
    pin_pwm_.max = pwm + pin_pwm_.offset;
  }
  void setOffset(float offset) { // offset: [0, 1]
    pin_pwm_.offset = offset;
  }

  void on();
  void off();

private:
  PwmPin pin_pwm_;
  Pin pin_in1_;
  Pin pin_in2_;

  bool isOn_;
};

} /* namespace robot */

#endif /* ROBOT_MOTOR_H_ */
