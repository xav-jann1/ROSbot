#ifndef ROBOT_NODE_H_
#define ROBOT_NODE_H_

#include "ros.h"
#include "std_msgs/Float64MultiArray.h"

#include "robot_config.h"
#include "robot_VelocityJoint.h"

namespace robot {

void initNode(ros::NodeHandle& nh);
void initJointsDataMsg();

void updateNode(ros::NodeHandle& nh);
void publishJointsData();

}
#endif /* ROBOT_NODE_H_ */
