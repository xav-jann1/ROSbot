#ifndef ROBOT_JOINT_H_
#define ROBOT_JOINT_H_

#include "robot/node/VelocityJointWithTopics.h"
#include "ros.h"
#include <string>

namespace robot {

void initJoint(robot::VelocityJointWithTopics &joint, ros::NodeHandle& nh);

void getJointPidValues(robot::VelocityJointWithTopics &joint, std::string topic_name, ros::NodeHandle& nh);

}

#endif /* ROBOT_JOINT_H_ */
