#include "robot_VelocityJoint.h"

namespace robot {

VelocityJoint::VelocityJoint(EncoderDef enc_def, PidDef pid_def, MotorDef mot_def, JointDef joint_def) :
    encoder_(enc_def, joint_def.name + "/encoder"),
    motor_(mot_def),
    pid_(pid_def, joint_def.name + "/pid"),
    cmd_vel_(0), ratio_(joint_def.ratio),

    set_command_name_(joint_def.name + "/command"),
    set_command_sub_(set_command_name_.c_str(), &VelocityJoint::set_command_cb, this),
    on_off_name_(joint_def.name + "/on_off"),
    on_off_sub_(on_off_name_.c_str(), &VelocityJoint::on_off_cb, this),

    pos_name_(joint_def.name + "/pos"), vel_name_(joint_def.name + "/vel"), pwm_name_(joint_def.name + "/pwm"),
    pos_pub_(pos_name_.c_str(), &pos_msg_),
    vel_pub_(vel_name_.c_str(), &vel_msg_),
    pwm_pub_(pwm_name_.c_str(), &pwm_msg_) {
}

void VelocityJoint::init() {
  encoder_.init();
  motor_.init();
  getDeltaTicks();
}

void VelocityJoint::addJointSubscriber(ros::NodeHandle& nh) {
  nh.subscribe(set_command_sub_);
  nh.subscribe(on_off_sub_);
}

void VelocityJoint::addJointPublishers(ros::NodeHandle& nh) {
  nh.advertise(pos_pub_);
  nh.advertise(vel_pub_);
  nh.advertise(pwm_pub_);
}

void VelocityJoint::publishData() {
  pos_msg_.data = getPosition();
  vel_msg_.data = getVelocity();
  pwm_msg_.data = pwm_;

  pos_pub_.publish(&pos_msg_);
  vel_pub_.publish(&vel_msg_);
  pwm_pub_.publish(&pwm_msg_);
}

float VelocityJoint::update() {
  float dt = getDeltaTicks() / 1000.0f;

  // Encoder:
  encoder_.update(dt);
  float vel = encoder_.getVelocity() * ratio_;

  // PID:
  float effort = pid_.updatePid(vel - cmd_vel_, dt);

  // Motor:
  pwm_ = effort;
  motor_.turn(pwm_);

  return pwm_;
}

float VelocityJoint::getDeltaTicks() {
  static uint32_t t_;
  uint32_t t = HAL_GetTick();
  uint32_t dt = t - t_;
  t_ = t;
  return dt;
}

} /* namespace robot */
