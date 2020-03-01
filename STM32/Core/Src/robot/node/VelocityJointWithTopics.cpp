#include "robot/node/VelocityJointWithTopics.h"

namespace robot {

VelocityJointWithTopics::VelocityJointWithTopics(EncoderDef enc_def, PidDef pid_def, MotorDef mot_def, JointDef joint_def) :
    VelocityJointAbstract(&encoder_, &pid_, &motor_, joint_def.ratio),
    encoder_(enc_def, joint_def.name + "/encoder"),
    pid_(pid_def, joint_def.name + "/pid"),
    motor_(mot_def),

    set_command_sub_(joint_def.name + "/command", &VelocityJointWithTopics::set_command_cb, this),
    on_off_sub_(joint_def.name + "/on_off", &VelocityJointWithTopics::on_off_cb, this),

    pos_pub_(joint_def.name + "/pos"),
    vel_pub_(joint_def.name + "/vel"),
    pwm_pub_(joint_def.name + "/pwm")
{
}

void VelocityJointWithTopics::addJointSubscriber(ros::NodeHandle& nh) {
  set_command_sub_.subscribe(nh);
  on_off_sub_.subscribe(nh);
}

void VelocityJointWithTopics::addJointPublishers(ros::NodeHandle& nh) {
  pos_pub_.advertise(nh);
  vel_pub_.advertise(nh);
  pwm_pub_.advertise(nh);
}

void VelocityJointWithTopics::publishData(){
  pos_pub_.publish(getPosition());
  vel_pub_.publish(getVelocity());
  pwm_pub_.publish(pwm_);
}


} /* namespace robot */
