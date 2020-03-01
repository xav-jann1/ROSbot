#ifndef INC_ROBOT_NODE_TOPICPUBLISHER_H_
#define INC_ROBOT_NODE_TOPICPUBLISHER_H_

#include "ros.h"
#include <string>

namespace robot {

/**
 * Wrapper de ros::Publisher pour simplifier l'envoi et enregistrer le nom du topic,
 * sinon rien ne garantit que la valeur est gardée en mémoire
 */
template<typename MsgT>
class TopicPublisher : public ros::Publisher {
public:
  TopicPublisher(std::string name) :
    ros::Publisher("", &msg_),
    name_(name)
  {
    this->topic_ = name_.c_str();
  }

  void advertise(ros::NodeHandle& nh) {
    nh.advertise(*this);
  }

  template<typename T>
  void publish(T data) {
    msg_.data = data;
    Publisher::publish(&msg_);
  }

private:
  std::string name_;
  MsgT msg_;
};

}

#endif /* INC_ROBOT_NODE_TOPICPUBLISHER_H_ */
