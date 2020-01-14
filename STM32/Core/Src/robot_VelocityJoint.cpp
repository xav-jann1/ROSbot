#include "robot_VelocityJoint.h"

namespace robot {

VelocityJoint::VelocityJoint(EncoderDef enc_def, PidDef pid_def, MotorDef mot_def, JointDef joint_def) :
    encoder_(enc_def, joint_def.name + "/encoder"),
    motor_(mot_def),
    pid_(pid_def, joint_def.name + "/pid"),
    cmd_vel_(0), ratio_(joint_def.ratio) {
}

void VelocityJoint::init() {
  encoder_.init();
  motor_.init();
  getDeltaTicks();
}

float VelocityJoint::update() {
  float dt = getDeltaTicks() / 1000.0f;

  // Encoder:
  encoder_.update(dt);
  float vel = encoder_.getVelocity() * ratio_;

  // PID:
  float effort = pid_.updatePid(vel - cmd_vel_, dt);

  // Motor:
  float pwm = effort;
  motor_.turn(pwm);

  return pwm;
}

float VelocityJoint::getDeltaTicks() {
  static uint32_t t_;
  uint32_t t = HAL_GetTick();
  uint32_t dt = t - t_;
  t_ = t;
  return dt;
}

} /* namespace robot */
