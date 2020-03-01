#ifndef ROBOT_PIDWITHTOPICS_H
#define ROBOT_PIDWITHTOPICS_H

#include "robot/PID.h"
#include <string>

#include "ros.h"
#include "robot/node/TopicSubscriber.h"
#include "robot/node/TopicPublisher.h"
#include "std_msgs/Float32.h"

namespace robot {

class PidWithTopics : public Pid {
public:
  PidWithTopics(double P = 0.0, double I = 0.0, double D = 0.0, double I1 = 0.0, double I2 = -0.0, std::string name = "pid");
  PidWithTopics(PidDef pid_def, std::string name = "pid");

  void initPid(ros::NodeHandle&, const std::string);
  void addSubscribers(ros::NodeHandle&);
  void addPublishers(ros::NodeHandle&);
  void publishPidValues();

private:
  // Subscribers:
  TopicSubscriber<std_msgs::Float32, robot::PidWithTopics> set_p_sub_;
  TopicSubscriber<std_msgs::Float32, robot::PidWithTopics> set_i_sub_;
  TopicSubscriber<std_msgs::Float32, robot::PidWithTopics> set_d_sub_;
  TopicSubscriber<std_msgs::Float32, robot::PidWithTopics> set_i_max_sub_;
  TopicSubscriber<std_msgs::Float32, robot::PidWithTopics> set_i_min_sub_;

  // Subscribers' Callback:
  void set_p_cb(const std_msgs::Float32 &msg) { p_gain_ = msg.data; }
  void set_i_cb(const std_msgs::Float32 &msg) { i_gain_ = msg.data; }
  void set_d_cb(const std_msgs::Float32 &msg) { d_gain_ = msg.data; }
  void set_i_max_cb(const std_msgs::Float32 &msg) { i_max_ = msg.data; }
  void set_i_min_cb(const std_msgs::Float32 &msg) { i_min_ = msg.data; }

  // Publishers:
  TopicPublisher<std_msgs::Float32> get_p_pub_;
  TopicPublisher<std_msgs::Float32> get_i_pub_;
  TopicPublisher<std_msgs::Float32> get_d_pub_;
  TopicPublisher<std_msgs::Float32> get_i_max_pub_;
  TopicPublisher<std_msgs::Float32> get_i_min_pub_;
};

}

#endif /* ROBOT_PIDWITHTOPICS_H */
