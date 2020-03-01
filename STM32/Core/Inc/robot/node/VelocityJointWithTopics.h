#ifndef ROBOT_VELOCITYJOINTWITHTOPICS_H_
#define ROBOT_VELOCITYJOINTWITHTOPICS_H_

#include <string>
#include "robot/VelocityJoint.h"
#include "robot/node/EncoderWithTopics.h"
#include "robot/node/PIDwithTopics.h"
#include "robot/Motor.h"

#include "ros.h"
#include "robot/node/TopicSubscriber.h"
#include "robot/node/TopicPublisher.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

namespace robot {

class VelocityJointWithTopics : public VelocityJointAbstract {
public:
  VelocityJointWithTopics(EncoderDef, PidDef, MotorDef, JointDef);

  // Encoder:
  void addEncoderPublishers(ros::NodeHandle& nh) {
    encoder_.addPublishers(nh);
  }
  void addEncoderSubscriber(ros::NodeHandle& nh) {
      encoder_.addSubscriber(nh);
    }
  void publishEncoderData() {
    encoder_.publishData();
  }

  // PID:
  void initPid(ros::NodeHandle& nh, const std::string prefix) {
    pid_.initPid(nh, prefix);
  }
  void addPidSubscribers(ros::NodeHandle& nh) {
    pid_.addSubscribers(nh);
  }
  void addPidPublishers(ros::NodeHandle& nh) {
    pid_.addPublishers(nh);
  }
  void publishPidValues() {
    pid_.publishPidValues();
  }

  // Joint:
  void addJointSubscriber(ros::NodeHandle& nh);
  void addJointPublishers(ros::NodeHandle& nh);
  void publishData();


private:
  EncoderWithTopics encoder_;
  PidWithTopics pid_;
  Motor motor_;

  // Subscribers:
  TopicSubscriber<std_msgs::Float32, robot::VelocityJointWithTopics> set_command_sub_;
  void set_command_cb(const std_msgs::Float32 &msg) { cmd_vel_ = msg.data; }

  TopicSubscriber<std_msgs::Bool, robot::VelocityJointWithTopics> on_off_sub_;
  void on_off_cb(const std_msgs::Bool &msg) {
    msg.data ? motor_.on() : motor_.off();
  }

  // Publishers:
  TopicPublisher<std_msgs::Float32> pos_pub_;
  TopicPublisher<std_msgs::Float32> vel_pub_;
  TopicPublisher<std_msgs::Float32> pwm_pub_;
};

} /* namespace robot */

#endif /* ROBOT_VELOCITYJOINTWITHTOPICS_H_ */
