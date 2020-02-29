#ifndef ROBOT_ENCODERWITHTOPICS_H_
#define ROBOT_ENCODERWITHTOPICS_H_

#include "robot/Encoder.h"
#include <string>

#include "ros.h"
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
  // Noms des topics des publishers:
  std::string pos_name_;
  std::string vel_name_;
  std::string ticks_name_;
  std::string dticks_name_;

  // Publishers:
  ros::Publisher pos_pub_;
  ros::Publisher vel_pub_;
  ros::Publisher ticks_pub_;
  ros::Publisher dticks_pub_;

  // Publishers' message:
  std_msgs::Float32 pos_msg_;
  std_msgs::Float32 vel_msg_;
  std_msgs::Int32 ticks_msg_;
  std_msgs::Int32 dticks_msg_;

  // Subscriber:
  std::string reset_name_;
  ros::Subscriber<std_msgs::Empty, robot::EncoderWithTopics> reset_sub_;
  void reset_cb(const std_msgs::Empty &msg) { ticks_ = dticks_ = dticks_s_ = 0; }
};

} /* namespace robot */

#endif /* ROBOT_ENCODERWITHTOPICS_H_ */
