#ifndef ROBOT_JOINT_H_
#define ROBOT_JOINT_H_

#include <string>
#include "ros.h"
#include "robot/VelocityJoint.h"

namespace robot {

void initJoint(robot::VelocityJoint &joint, ros::NodeHandle& nh);

void getJointPidValues(robot::VelocityJoint &joint, std::string topic_name, ros::NodeHandle& nh);

}

#endif /* ROBOT_JOINT_H_ */
