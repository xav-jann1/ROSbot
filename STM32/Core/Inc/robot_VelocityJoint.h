#ifndef ROBOT_VELOCITYJOINT_H_
#define ROBOT_VELOCITYJOINT_H_

#include <string>
#include "robot_PID.h"
#include "robot_Motor.h"
#include "robot_Encoder.h"

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
  void publishEncoderData() {
    encoder_.publishData();
  }

  // PID:
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
  float getVelocity() {
    return encoder_.getVelocity() * ratio_;
  }
  float getPosition() {
    return encoder_.getPosition() * ratio_;
  }

private:
  Encoder encoder_;
  Motor motor_;
  Pid pid_;

  float cmd_vel_ = 0;
  float ratio_ = 1;

  float getDeltaTicks();
};

} /* namespace robot */

#endif /* ROBOT_VELOCITYJOINT_H_ */
