#include "RevoluteJoint.h"

#include <JointsUtil.h>

#include "Constraint1D.h"
#include "Constraint1DContainer.h"

void physecs::RevoluteJoint::makeConstraints(const JointWorldSpaceData &worldSpaceData, void *additionalData, Constraint1DDescriptor* constraintRows) {
    auto& [p0, p1, r0, r1, u0, u1] = worldSpaceData;
    auto& [driveEnabled, driveVelocity, driveMaxTorque] = *static_cast<RevoluteJointDef::Data*>(additionalData);

    const float d01 = glm::dot(u0[0], u1[1]);
    const glm::vec3 u11xu00 = glm::cross(u1[1], u0[0]);

    constraintRows[0].angular0 = u11xu00;
    constraintRows[0].angular1 = u11xu00;
    constraintRows[0].geometricError = d01;

    const float d02 = glm::dot(u0[0], u1[2]);
    const glm::vec3 u12xu00 = glm::cross(u1[2], u0[0]);

    constraintRows[1].angular0 = u12xu00;
    constraintRows[1].angular1 = u12xu00;
    constraintRows[1].geometricError = d02;

    createPointToPointConstraint(p0, p1, r0, r1, constraintRows + 2);

    if (driveEnabled) {
        constraintRows[5].angular0 = u0[0];
        constraintRows[5].angular1 = u0[0];
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
    PHYSECS_DECLARE_JOINT_IMPL(RevoluteJoint);
}
