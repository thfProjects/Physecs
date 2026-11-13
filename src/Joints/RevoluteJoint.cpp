#include "RevoluteJoint.h"
#include "JointsUtil.h"
#include "Joint.inl"

void physecs::RevoluteJoint::makeConstraints(const JointWorldSpaceData &worldSpaceData, void *additionalData, Constraint1DDescriptor* constraintRows) {
    auto& [p0, p1, r0, r1, q0, q1] = worldSpaceData;
    auto& [driveEnabled, driveVelocity, driveMaxTorque] = *static_cast<RevoluteJointDef::Data*>(additionalData);

    // constraining relative rotation q = conj(q0) * q1, which is local joint frame 0, to be pure rotation around x axis
    {
        const glm::quat q = q0 * glm::quat(q1.y, -q1.z, q1.w, q1.x);
        constraintRows[0].angular0 = glm::vec3(q.x, q.y, q.z);
        constraintRows[0].angular1 = glm::vec3(q.x, q.y, q.z);
        constraintRows[0].geometricError = 2.f * q.w;
    }

    {
        const glm::quat q = q0 * glm::quat(q1.z, q1.y, -q1.x, q1.w);
        constraintRows[1].angular0 = glm::vec3(q.x, q.y, q.z);
        constraintRows[1].angular1 = glm::vec3(q.x, q.y, q.z);
        constraintRows[1].geometricError = 2.f * q.w;
    }

    createPointToPointConstraint(p0, p1, r0, r1, constraintRows + 2);

    if (driveEnabled) {
        // x axis of joint frame 0, first column of rotation matrix for quaternion q0
        const glm::vec3 hingeAxis(
            1.f - 2.f * (q0.y * q0.y + q0.z * q0.z),
            2.f * (q0.x * q0.y + q0.w * q0.z),
            2.f * (q0.x * q0.z - q0.w * q0.y));

        constraintRows[5].angular0 = hingeAxis;
        constraintRows[5].angular1 = hingeAxis;
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
