#include "ServoJoint.h"

#include <JointsUtil.h>

#include "Constraint1D.h"
#include <glm/gtx/vector_angle.hpp>
#include "Constraint1DContainer.h"

void physecs::ServoJoint::makeConstraints(JointWorldSpaceData &worldSpaceData, void *additionalData, Constraint1DWriter& constraints) {
    auto& [p0, p1, r0, r1, u0, u1] = worldSpaceData;
    auto& [targetAngle, driveStiffness, driveDamping] = *static_cast<ServoJointData*>(additionalData);

    createPointToPointConstraint(p0, p1, r0, r1, constraints);

    const float d01 = glm::dot(u0[0], u1[1]);
    const glm::vec3 u11xu00 = glm::cross(u1[1], u0[0]);

    constraints.next<ANGULAR>()
    .setAngular0(u11xu00)
    .setAngular1(u11xu00)
    .setC(d01);

    const float d02 = glm::dot(u0[0], u1[2]);
    const glm::vec3 u12xu00 = glm::cross(u1[2], u0[0]);

    constraints.next<ANGULAR>()
    .setAngular0(u12xu00)
    .setAngular1(u12xu00)
    .setC(d02);

    constraints.next<ANGULAR | SOFT>()
    .setAngular0(u0[0])
    .setAngular1(u0[0])
    .setC(glm::orientedAngle(u0[2], u1[2], u0[0]) - targetAngle)
    .setFrequency(driveStiffness)
    .setDampingRatio(driveDamping);
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

physecs::JointSolverDesc physecs::ServoJoint::getSolverDesc(entt::registry &registry, Constraint1DLayout& constraintLayout) {
    constraintLayout.createConstraints<NONE, 3>();
    constraintLayout.createConstraints<ANGULAR, 2>();
    constraintLayout.createConstraints<ANGULAR | SOFT>();
    return {
        &data,
        makeConstraints
    };
}
