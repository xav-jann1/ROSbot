#ifndef INC_ROBOT_NODE_TOPICSUBSCRIBER_H_
#define INC_ROBOT_NODE_TOPICSUBSCRIBER_H_

#include "ros.h"
#include <string>

namespace robot {

/**
 * Wrapper de ros::Subscriber pour enregistrer le nom du topic,
 * sinon rien ne garantit que la valeur est gardée en mémoire
 */
template<typename MsgT, typename ObjT = void>
class TopicSubscriber : public ros::Subscriber<MsgT, ObjT> {
public:
  typedef void(ObjT::*CallbackT)(const MsgT&);
  TopicSubscriber(std::string name, CallbackT cb, ObjT* obj) :
    ros::Subscriber<MsgT, ObjT>("", cb, obj),
    name_(name)
  {
    this->topic_ = name_.c_str();
  }

  void subscribe(ros::NodeHandle& nh) {
    nh.subscribe(*this);
  }

private:
  std::string name_;
};

} /* namespace robot */

#endif /* INC_ROBOT_NODE_TOPICSUBSCRIBER_H_ */
