#include "robot/VelocityJoint.h"

namespace robot {

VelocityJoint::VelocityJoint(EncoderDef enc_def, PidDef pid_def, MotorDef mot_def, JointDef joint_def) :
    VelocityJointAbstract(&encoder_, &pid_, &motor_, joint_def.ratio),
    encoder_(enc_def),
    pid_(pid_def),
    motor_(mot_def)
{
}

} /* namespace robot */
