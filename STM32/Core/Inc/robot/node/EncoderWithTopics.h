#ifndef ROBOT_ENCODERWITHTOPICS_H_
#define ROBOT_ENCODERWITHTOPICS_H_

#include "robot/Encoder.h"
#include <string>

#include "ros.h"
#include "robot/node/TopicSubscriber.h"
#include "robot/node/TopicPublisher.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Empty.h"

namespace robot {

class EncoderWithTopics : public Encoder {
public:
  EncoderWithTopics(EncoderDef, std::string = "encoder");

  // Publisher:
  void addPublishers(ros::NodeHandle& nh);
  void publishData();

  // Subscriber
  void addSubscriber(ros::NodeHandle& nh);

private:
  // Publishers:
  TopicPublisher<std_msgs::Float32> pos_pub_;
  TopicPublisher<std_msgs::Float32> vel_pub_;
  TopicPublisher<std_msgs::Int32> ticks_pub_;
  TopicPublisher<std_msgs::Int32> dticks_pub_;

  // Subscriber:
  TopicSubscriber<std_msgs::Empty, robot::EncoderWithTopics> reset_sub_;
  void reset_cb(const std_msgs::Empty &msg) { ticks_ = dticks_ = dticks_s_ = 0; }
};

} /* namespace robot */

#endif /* ROBOT_ENCODERWITHTOPICS_H_ */
