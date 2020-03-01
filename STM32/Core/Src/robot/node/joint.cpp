#include "robot/node/joint.h"

namespace robot {

void initJoint(robot::VelocityJointWithTopics &joint, ros::NodeHandle& nh) {
  // Initialisation:
  joint.init();

  // Publishers, Subscribers, Service:
  joint.addJointPublishers(nh);
  joint.addJointSubscriber(nh);
  joint.addPidSubscribers(nh);
  joint.addPidPublishers(nh);
  joint.addEncoderPublishers(nh);
  joint.addEncoderSubscriber(nh);
}

void getJointPidValues(robot::VelocityJointWithTopics &joint, std::string topic_name, ros::NodeHandle& nh) {
  joint.initPid(nh, topic_name.c_str());
}

}
