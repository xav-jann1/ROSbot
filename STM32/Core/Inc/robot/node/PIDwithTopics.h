#ifndef ROBOT_PIDWITHTOPICS_H
#define ROBOT_PIDWITHTOPICS_H

#include "robot/PID.h"
#include <string>

#include "ros.h"
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
  // Subscribers' name:
  std::string set_p_name_;
  std::string set_i_name_;
  std::string set_d_name_;
  std::string set_i_max_name_;
  std::string set_i_min_name_;

  // Subscribers:
  ros::Subscriber<std_msgs::Float32, robot::PidWithTopics> set_p_sub_;
  ros::Subscriber<std_msgs::Float32, robot::PidWithTopics> set_i_sub_;
  ros::Subscriber<std_msgs::Float32, robot::PidWithTopics> set_d_sub_;
  ros::Subscriber<std_msgs::Float32, robot::PidWithTopics> set_i_max_sub_;
  ros::Subscriber<std_msgs::Float32, robot::PidWithTopics> set_i_min_sub_;

  // Subscribers' Callback:
  void set_p_cb(const std_msgs::Float32 &msg) { p_gain_ = msg.data; }
  void set_i_cb(const std_msgs::Float32 &msg) { i_gain_ = msg.data; }
  void set_d_cb(const std_msgs::Float32 &msg) { d_gain_ = msg.data; }
  void set_i_max_cb(const std_msgs::Float32 &msg) { i_max_ = msg.data; }
  void set_i_min_cb(const std_msgs::Float32 &msg) { i_min_ = msg.data; }

  // Publishers" name:
  std::string get_p_name_;
  std::string get_i_name_;
  std::string get_d_name_;
  std::string get_i_max_name_;
  std::string get_i_min_name_;

  // Publishers:
  ros::Publisher get_p_pub_;
  ros::Publisher get_i_pub_;
  ros::Publisher get_d_pub_;
  ros::Publisher get_i_max_pub_;
  ros::Publisher get_i_min_pub_;

  // Publishers' message:
  std_msgs::Float32 get_p_msg_;
  std_msgs::Float32 get_i_msg_;
  std_msgs::Float32 get_d_msg_;
  std_msgs::Float32 get_i_max_msg_;
  std_msgs::Float32 get_i_min_msg_;
};

}

#endif /* ROBOT_PIDWITHTOPICS_H */
