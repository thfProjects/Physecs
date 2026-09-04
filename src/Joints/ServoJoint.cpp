#include "ServoJoint.h"

#include <JointsUtil.h>

#include "Constraint1D.h"
#include <glm/gtx/vector_angle.hpp>
#include "Constraint1DContainer.h"

void physecs::ServoJoint::makeConstraints(const JointWorldSpaceData &worldSpaceData, void *additionalData, Constraint1DDescriptor* constraintRows) {
    auto& [p0, p1, r0, r1, u0, u1] = worldSpaceData;
    auto& [targetAngle, driveStiffness, driveDamping] = *static_cast<ServoJointDef::Data*>(additionalData);

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

    constraintRows[5].angular0 = u0[0];
    constraintRows[5].angular1 = u0[0];
    constraintRows[5].geometricError = glm::orientedAngle(u0[2], u1[2], u0[0]) - targetAngle;
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
