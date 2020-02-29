#include "robot/node/VelocityJointWithTopics.h"

namespace robot {

VelocityJointWithTopics::VelocityJointWithTopics(EncoderDef enc_def, PidDef pid_def, MotorDef mot_def, JointDef joint_def) :
    VelocityJointAbstract(&encoder_, &pid_, &motor_, joint_def.ratio),
    encoder_(enc_def, joint_def.name + "/encoder"),
    pid_(pid_def, joint_def.name + "/pid"),
    motor_(mot_def),

    set_command_name_(joint_def.name + "/command"),
    set_command_sub_(set_command_name_.c_str(), &VelocityJointWithTopics::set_command_cb, this),
    on_off_name_(joint_def.name + "/on_off"),
    on_off_sub_(on_off_name_.c_str(), &VelocityJointWithTopics::on_off_cb, this),

    pos_name_(joint_def.name + "/pos"), vel_name_(joint_def.name + "/vel"), pwm_name_(joint_def.name + "/pwm"),
    pos_pub_(pos_name_.c_str(), &pos_msg_),
    vel_pub_(vel_name_.c_str(), &vel_msg_),
    pwm_pub_(pwm_name_.c_str(), &pwm_msg_)
{

}

void VelocityJointWithTopics::addJointSubscriber(ros::NodeHandle& nh) {
  nh.subscribe(set_command_sub_);
  nh.subscribe(on_off_sub_);
}

void VelocityJointWithTopics::addJointPublishers(ros::NodeHandle& nh) {
  nh.advertise(pos_pub_);
  nh.advertise(vel_pub_);
  nh.advertise(pwm_pub_);
}

void VelocityJointWithTopics::publishData() {
  pos_msg_.data = getPosition();
  vel_msg_.data = getVelocity();
  pwm_msg_.data = pwm_;

  pos_pub_.publish(&pos_msg_);
  vel_pub_.publish(&vel_msg_);
  pwm_pub_.publish(&pwm_msg_);
}



} /* namespace robot */
