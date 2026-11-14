#include "RevoluteJoint.h"
#include "JointsUtil.h"
#include "Joint.inl"

void physecs::RevoluteJoint::makeConstraints(const JointWorldSpaceData &worldSpaceData, void *additionalData, Constraint1DDescriptor* constraintRows) {
    auto& [p0, p1, r0, r1, q0, q1] = worldSpaceData;
    auto& [driveEnabled, driveVelocity, driveMaxTorque] = *static_cast<RevoluteJointDef::Data*>(additionalData);

    createAngularHingeConstraint(q0, q1, constraintRows);
    createPointToPointConstraint(p0, p1, r0, r1, constraintRows + 2);

    if (driveEnabled) {
        // x axis of joint frame 0, first column of rotation matrix for quaternion q0
        const glm::vec3 axis = col0(q0);

        constraintRows[5].angular0 = axis;
        constraintRows[5].angular1 = axis;
        constraintRows[5].targetVelocity = driveVelocity;
        constraintRows[5].minForce = -driveMaxTorque;
        constraintRows[5].maxForce = driveMaxTorque;
    }
}

void physecs::RevoluteJoint::setDriveEnabled(bool enabled) {
    data.driveEnabled = enabled;
}

void physecs::RevoluteJoint::setDriveVelocity(float velocity) {
    data.driveVelocity = velocity;
}

void physecs::RevoluteJoint::setDriveMaxTorque(float maxTorque) {
    data.driveMaxTorque = maxTorque;
}

namespace physecs {
    PHYSECS_DEFINE_JOINT_IMPL(RevoluteJoint);
}
