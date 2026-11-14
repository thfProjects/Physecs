#include "ServoJoint.h"
#include "JointsUtil.h"
#include "MathUtil.h"
#include "Joint.inl"

void physecs::ServoJoint::makeConstraints(const JointWorldSpaceData &worldSpaceData, void *additionalData, Constraint1DDescriptor* constraintRows) {
    auto& [p0, p1, r0, r1, q0, q1] = worldSpaceData;
    auto& [targetAngle, driveStiffness, driveDamping] = *static_cast<ServoJointDef::Data*>(additionalData);

    createAngularHingeConstraint(q0, q1, constraintRows); 
    createPointToPointConstraint(p0, p1, r0, r1, constraintRows + 2);

    // w and x components of the relative rotation q = conj(q0) * q1, the hinge angle is 2 * atan2(q.x, q.w)
    const float qw = q0.w * q1.w + q0.x * q1.x + q0.y * q1.y + q0.z * q1.z;
    const float qx = q0.w * q1.x - q0.x * q1.w - q0.y * q1.z + q0.z * q1.y;

    // world space hinge axis, column 0 of the rotation matrix of q0
    const glm::vec3 axis = col0(q0); 

    // negating qx and qw keeps the same rotation, and ensuring qw is positive keeps the output of atan in the [-pi/2, pi/2] range
    float s = qw < 0 ? -1.f : 1.f;
    const float angle = 2.f * glm::atan(s * qx, s * qw);

    constraintRows[5].angular0 = axis;
    constraintRows[5].angular1 = axis;
    constraintRows[5].geometricError = angle - targetAngle;
    constraintRows[5].stiffness = driveStiffness;
    constraintRows[5].damping = driveDamping;
}

void physecs::ServoJoint::setTargetAngle(float angle) {
    data.targetAngle = angle;
}

void physecs::ServoJoint::setDriveStiffness(float stiffness) {
    data.driveStiffness = stiffness;
}

void physecs::ServoJoint::setDriveDamping(float damping) {
    data.driveDamping = damping;
}

namespace physecs {
    PHYSECS_DEFINE_JOINT_IMPL(ServoJoint);
}
