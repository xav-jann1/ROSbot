#include "robot/node/PIDwithTopics.h"

namespace robot {

PidWithTopics::PidWithTopics(double P, double I, double D, double I1, double I2, std::string name) :
  Pid(P, I, D, I1, I2),

  set_p_sub_(name + "/set_p", &PidWithTopics::set_p_cb, this),
  set_i_sub_(name + "/set_i", &PidWithTopics::set_i_cb, this),
  set_d_sub_(name + "/set_d", &PidWithTopics::set_d_cb, this),
  set_i_max_sub_(name + "/set_i_max", &PidWithTopics::set_i_max_cb, this),
  set_i_min_sub_(name + "/set_i_min", &PidWithTopics::set_i_min_cb, this),

  get_p_pub_(name + "/get_p"),
  get_i_pub_(name + "/get_i"),
  get_d_pub_(name + "/get_d"),
  get_i_max_pub_(name + "/get_i_max"),
  get_i_min_pub_(name + "/get_i_min")
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
  set_p_sub_.subscribe(nh);
  set_i_sub_.subscribe(nh);
  set_d_sub_.subscribe(nh);
  set_i_max_sub_.subscribe(nh);
  set_i_min_sub_.subscribe(nh);
}

void PidWithTopics::addPublishers(ros::NodeHandle& nh) {
  get_p_pub_.advertise(nh);
  get_i_pub_.advertise(nh);
  get_d_pub_.advertise(nh);
  get_i_max_pub_.advertise(nh);
  get_i_min_pub_.advertise(nh);
}

void PidWithTopics::publishPidValues() {
  get_p_pub_.publish(p_gain_);
  get_i_pub_.publish(i_gain_);
  get_d_pub_.publish(d_gain_);
  get_i_max_pub_.publish(i_max_);
  get_i_min_pub_.publish(i_min_);
}

} /* namespace robot */
