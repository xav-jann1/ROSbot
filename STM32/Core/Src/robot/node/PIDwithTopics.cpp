#include "robot/node/PIDwithTopics.h"

namespace robot {

PidWithTopics::PidWithTopics(double P, double I, double D, double I1, double I2, std::string name) :
  Pid(P, I, D, I1, I2),

  set_p_name_(name + "/set_p"),
  set_i_name_(name + "/set_i"),
  set_d_name_(name + "/set_d"),
  set_i_max_name_(name + "/set_i_max"),
  set_i_min_name_(name + "/set_i_min"),
  set_p_sub_(set_p_name_.c_str(), &PidWithTopics::set_p_cb, this),
  set_i_sub_(set_i_name_.c_str(), &PidWithTopics::set_i_cb, this),
  set_d_sub_(set_d_name_.c_str(), &PidWithTopics::set_d_cb, this),
  set_i_max_sub_(set_i_max_name_.c_str(), &PidWithTopics::set_i_max_cb, this),
  set_i_min_sub_(set_i_min_name_.c_str(), &PidWithTopics::set_i_min_cb, this),

  get_p_name_(name + "/get_p"), get_i_name_(name + "/get_i"), get_d_name_(name + "/get_d"),
  get_i_max_name_(name + "/get_i_max"), get_i_min_name_(name + "/get_i_min"),
  get_p_pub_(get_p_name_.c_str(), &get_p_msg_),
  get_i_pub_(get_i_name_.c_str(), &get_i_msg_),
  get_d_pub_(get_d_name_.c_str(), &get_d_msg_),
  get_i_max_pub_(get_i_max_name_.c_str(), &get_i_max_msg_),
  get_i_min_pub_(get_i_min_name_.c_str(), &get_i_min_msg_)
{
  p_error_last_ = 0.0;
  p_error_ = 0.0;
  d_error_ = 0.0;
  i_error_ = 0.0;
  cmd_ = 0.0;
}

PidWithTopics::PidWithTopics(PidDef def, std::string name) : PidWithTopics(def.p, def.i, def.d, def.i_max, def.i_min, name)
{
}

void PidWithTopics::initPid(ros::NodeHandle& nh, const std::string prefix)
{
  float p, i, d, i_max, i_min;
  nh.loginfo(("> Récupère valeurs du PID dans '" + prefix + "'").c_str());
  nh.getParam((prefix + "/p").c_str(), &p);
  nh.getParam((prefix + "/i").c_str(), &i);
  nh.getParam((prefix + "/d").c_str(), &d);
  nh.getParam((prefix + "/i_max").c_str(), &i_max);
  nh.getParam((prefix + "/i_min").c_str(), &i_min);
  Pid::initPid(p, i, d, i_max, i_min);
}

void PidWithTopics::addSubscribers(ros::NodeHandle& nh) {
  nh.subscribe(set_p_sub_);
  nh.subscribe(set_i_sub_);
  nh.subscribe(set_d_sub_);
  nh.subscribe(set_i_max_sub_);
  nh.subscribe(set_i_min_sub_);
}

void PidWithTopics::addPublishers(ros::NodeHandle& nh) {
  nh.advertise(get_p_pub_);
  nh.advertise(get_i_pub_);
  nh.advertise(get_d_pub_);
  nh.advertise(get_i_max_pub_);
  nh.advertise(get_i_min_pub_);
}

void PidWithTopics::publishPidValues() {
  get_p_msg_.data = p_gain_;
  get_i_msg_.data = i_gain_;
  get_d_msg_.data = d_gain_;
  get_i_max_msg_.data = i_max_;
  get_i_min_msg_.data = i_min_;

  get_p_pub_.publish(&get_p_msg_);
  get_i_pub_.publish(&get_i_msg_);
  get_d_pub_.publish(&get_d_msg_);
  get_i_max_pub_.publish(&get_i_max_msg_);
  get_i_min_pub_.publish(&get_i_min_msg_);
}

} /* namespace robot */
