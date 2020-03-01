#ifndef ROBOT_VELOCITYJOINT_H_
#define ROBOT_VELOCITYJOINT_H_

#include "VelocityJointAbstract.h"

namespace robot {

typedef struct {
  std::string name;
  float ratio;
} JointDef;

class VelocityJoint : public VelocityJointAbstract {
public:
  VelocityJoint(EncoderDef, PidDef, MotorDef, JointDef);

private:
  Encoder encoder_;
  Pid pid_;
  Motor motor_;
};

} /* namespace robot */

#endif /* ROBOT_VELOCITYJOINT_H_ */
