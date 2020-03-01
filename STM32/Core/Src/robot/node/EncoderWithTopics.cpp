#include "robot/node/EncoderWithTopics.h"

namespace robot {

EncoderWithTopics::EncoderWithTopics(EncoderDef enc_def, std::string name) :
    Encoder(enc_def),

    pos_pub_(name + "/pos"),
    vel_pub_(name + "/vel"),
    ticks_pub_(name + "/ticks"),
    dticks_pub_(name + "/dticks"),
    reset_sub_(name + "/reset", &EncoderWithTopics::reset_cb, this)
{
}

void EncoderWithTopics::addPublishers(ros::NodeHandle& nh) {
  pos_pub_.advertise(nh);
  vel_pub_.advertise(nh);
  ticks_pub_.advertise(nh);
  dticks_pub_.advertise(nh);
}

void EncoderWithTopics::addSubscriber(ros::NodeHandle& nh) {
  reset_sub_.subscribe(nh);
}

void EncoderWithTopics::publishData() {
  pos_pub_.publish(getPosition());
  vel_pub_.publish(getVelocity());
  ticks_pub_.publish(ticks_);
  dticks_pub_.publish(dticks_);
}

} /* namespace robot */
