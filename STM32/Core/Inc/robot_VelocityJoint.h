#ifndef ROBOT_VELOCITYJOINT_H_
#define ROBOT_VELOCITYJOINT_H_

#include <string>
#include "robot_PID.h"
#include "robot_Motor.h"
#include "robot_Encoder.h"
#include "std_msgs/Bool.h"

namespace robot {

typedef struct {
  std::string name;
  float ratio;
} JointDef;

class VelocityJoint {
public:
  VelocityJoint(EncoderDef, PidDef, MotorDef, JointDef);

  void init();
  float update();
  void command(float cmd) {
    cmd_vel_ = cmd;
  }

  // Encoder:
  float getEncoderPosition() {
    return encoder_.getPosition();
  }
  float getEncoderVelocity() {
    return encoder_.getVelocity();
  }
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

  // Motor:
  void on() {
    motor_.on();
  }
  void off() {
    motor_.off();
  }

  // Joint:
  void addJointSubscriber(ros::NodeHandle& nh);
  void addJointPublishers(ros::NodeHandle& nh);
  void publishData();
  float getVelocity() {
    return encoder_.getVelocity() * ratio_;
  }
  float getPosition() {
    return encoder_.getPosition() * ratio_;
  }
  float getPWM() {
    return pwm_;
  }

private:
  Encoder encoder_;
  Motor motor_;
  Pid pid_;

  float cmd_vel_ = 0;
  float ratio_ = 1;
  float pwm_ = 0;

  uint32_t t_ = 0;

  // Subscribers:
  std::string set_command_name_;
  ros::Subscriber<std_msgs::Float32, robot::VelocityJoint> set_command_sub_;
  void set_command_cb(const std_msgs::Float32 &msg) { cmd_vel_ = msg.data; }

  std::string on_off_name_;
  ros::Subscriber<std_msgs::Bool, robot::VelocityJoint> on_off_sub_;
  void on_off_cb(const std_msgs::Bool &msg) {
    msg.data ? motor_.on() : motor_.off();
  }

  // Publishers'name:
  std::string pos_name_;
  std::string vel_name_;
  std::string pwm_name_;

  // Publishers:
  ros::Publisher pos_pub_;
  ros::Publisher vel_pub_;
  ros::Publisher pwm_pub_;

  // Publishers' message:
  std_msgs::Float32 pos_msg_;
  std_msgs::Float32 vel_msg_;
  std_msgs::Float32 pwm_msg_;

  float getDeltaTicks();
};

} /* namespace robot */

#endif /* ROBOT_VELOCITYJOINT_H_ */
