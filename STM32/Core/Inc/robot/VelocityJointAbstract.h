#ifndef ROBOT_VELOCITYJOINTABSTRACT_H_
#define ROBOT_VELOCITYJOINTABSTRACT_H_

#include <string>
#include "PID.h"
#include "Motor.h"
#include "Encoder.h"

namespace robot {

class VelocityJointAbstract {
public:
  VelocityJointAbstract(Encoder*, Pid*, Motor*, float);

  void init();
  float update();
  void command(float cmd) {
    cmd_vel_ = cmd;
  }

  // Encoder:
  float getEncoderPosition() {
    return encoder_->getPosition();
  }
  float getEncoderVelocity() {
    return encoder_->getVelocity();
  }

  // Motor:
  void on() {
    motor_->on();
  }
  void off() {
    motor_->off();
  }

  // Joint:
  float getVelocity() {
    return encoder_->getVelocity() * ratio_;
  }
  float getPosition() {
    return encoder_->getPosition() * ratio_;
  }
  float getPWM() {
    return pwm_;
  }

protected:
  Encoder* encoder_;
  Pid* pid_;
  Motor* motor_;

  float cmd_vel_ = 0;
  float ratio_ = 1;
  float pwm_ = 0;

  uint32_t t_ = 0;

  float getDeltaTicks();
};

} /* namespace robot */

#endif /* ROBOT_VELOCITYJOINTABSTRACT_H_ */
