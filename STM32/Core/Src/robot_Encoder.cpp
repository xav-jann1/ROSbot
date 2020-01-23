#include "robot_Encoder.h"

namespace robot {

Encoder::Encoder(EncoderDef enc_def, std::string name) :
    htim_(enc_def.htim), resolution_(enc_def.resolution), tim_reload_(enc_def.tim_reload),
    pos_name_(name + "/pos"), vel_name_(name + "/vel"), ticks_name_(name + "/ticks"), dticks_name_(name + "/dticks"),
    pos_pub_(pos_name_.c_str(), &pos_msg_),
    vel_pub_(vel_name_.c_str(), &vel_msg_),
    ticks_pub_(ticks_name_.c_str(), &ticks_msg_),
    dticks_pub_(dticks_name_.c_str(), &dticks_msg_),
    reset_name_(name + "/reset"),
    reset_sub_(reset_name_.c_str(), &Encoder::reset_cb, this)
{
}

void Encoder::init() {
  if (HAL_TIM_Encoder_Start(&htim_, TIM_CHANNEL_1) != HAL_OK) {
    Init_Error_Handler();
  }
  if (HAL_TIM_Encoder_Start(&htim_, TIM_CHANNEL_2) != HAL_OK) {
    Init_Error_Handler();
  }

  resetTimerCount();
}

void Encoder::update(float dt_s) {
  int count = getTimerCount();
  resetTimerCount();

  dticks_ = count - tim_reload_;
  ticks_ += dticks_;
  dticks_s_ = dticks_ / dt_s;
}

void Encoder::addPublishers(ros::NodeHandle& nh) {
  nh.advertise(pos_pub_);
  nh.advertise(vel_pub_);
  nh.advertise(ticks_pub_);
  nh.advertise(dticks_pub_);
}

void Encoder::addSubscriber(ros::NodeHandle& nh) {
  nh.subscribe(reset_sub_);
}

void Encoder::publishData() {
  pos_msg_.data = getPosition();
  vel_msg_.data = getVelocity();
  ticks_msg_.data = ticks_;
  dticks_msg_.data = dticks_;

  pos_pub_.publish(&pos_msg_);
  vel_pub_.publish(&vel_msg_);
  ticks_pub_.publish(&ticks_msg_);
  dticks_pub_.publish(&dticks_msg_);
}

inline uint32_t Encoder::getTimerCount() {
  return htim_.Instance->CNT;
}

inline void Encoder::resetTimerCount() {
  __HAL_TIM_SET_COUNTER(&htim_, tim_reload_);
}

} /* namespace robot */
