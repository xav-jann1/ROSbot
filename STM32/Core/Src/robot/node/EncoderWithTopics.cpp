#include "robot/node/EncoderWithTopics.h"

namespace robot {

EncoderWithTopics::EncoderWithTopics(EncoderDef enc_def, std::string name) :
    Encoder(enc_def),

    pos_name_(name + "/pos"), vel_name_(name + "/vel"), ticks_name_(name + "/ticks"), dticks_name_(name + "/dticks"),
    pos_pub_(pos_name_.c_str(), &pos_msg_),
    vel_pub_(vel_name_.c_str(), &vel_msg_),
    ticks_pub_(ticks_name_.c_str(), &ticks_msg_),
    dticks_pub_(dticks_name_.c_str(), &dticks_msg_),

    reset_name_(name + "/reset"),
    reset_sub_(reset_name_.c_str(), &EncoderWithTopics::reset_cb, this)
{
}


void EncoderWithTopics::addPublishers(ros::NodeHandle& nh) {
  nh.advertise(pos_pub_);
  nh.advertise(vel_pub_);
  nh.advertise(ticks_pub_);
  nh.advertise(dticks_pub_);
}

void EncoderWithTopics::addSubscriber(ros::NodeHandle& nh) {
  nh.subscribe(reset_sub_);
}

void EncoderWithTopics::publishData() {
  pos_msg_.data = getPosition();
  vel_msg_.data = getVelocity();
  ticks_msg_.data = ticks_;
  dticks_msg_.data = dticks_;

  pos_pub_.publish(&pos_msg_);
  vel_pub_.publish(&vel_msg_);
  ticks_pub_.publish(&ticks_msg_);
  dticks_pub_.publish(&dticks_msg_);
}

} /* namespace robot */
