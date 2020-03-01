#include "robot/VelocityJointAbstract.h"

namespace robot {

VelocityJointAbstract::VelocityJointAbstract(Encoder* encoder, Pid* pid, Motor* motor, float ratio = 0) :
    encoder_(encoder),
    pid_(pid),
    motor_(motor)
{
}

void VelocityJointAbstract::init() {
  encoder_->init();
  motor_->init();
  getDeltaTicks();
}

float VelocityJointAbstract::update() {
  float dt = getDeltaTicks() / 1000.0f;

  // Encoder:
  encoder_->update(dt);
  float vel = encoder_->getVelocity() * ratio_;

  // PID:
  float effort = pid_->updatePid(vel - cmd_vel_, dt);

  // Motor:
  pwm_ = effort;
  motor_->turn(pwm_);

  return pwm_;
}

float VelocityJointAbstract::getDeltaTicks() {
  uint32_t t = HAL_GetTick();
  uint32_t dt = t - t_;
  t_ = t;
  return dt;
}

} /* namespace robot */
