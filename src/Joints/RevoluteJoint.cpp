#include "RevoluteJoint.h"
#include "Constraint1D.h"
#include "Constraint1DContainer.h"

void physecs::RevoluteJoint::makeConstraints(JointWorldSpaceData &worldSpaceData, void *additionalData, Constraint1DWriter& constraints) {
    auto& [p0, p1, r0, r1, u0, u1] = worldSpaceData;
    auto& [driveEnabled, driveVelocity, driveMaxTorque] = *static_cast<RevoluteJointData*>(additionalData);

    const glm::vec3 d = p1 - p0;

    const float cn = glm::dot(d, d);
    const glm::vec3 r0xd = glm::cross(r0, d);
    const glm::vec3 r1xd = glm::cross(r1, d);

    constraints.at(0)
    .setLinear(d)
    .setAngular0(r0xd)
    .setAngular1(r1xd)
    .setC(cn);

    const float d01 = glm::dot(u0[0], u1[1]);
    const glm::vec3 u11xu00 = glm::cross(u1[1], u0[0]);

    constraints.at<ANGULAR>(1)
    .setAngular0(u11xu00)
    .setAngular1(u11xu00)
    .setC(d01);

    const float d02 = glm::dot(u0[0], u1[2]);
    const glm::vec3 u12xu00 = glm::cross(u1[2], u0[0]);

    constraints.at<ANGULAR>(2)
    .setAngular0(u12xu00)
    .setAngular1(u12xu00)
    .setC(d02);

    if (driveEnabled) {
        constraints.at<ANGULAR | LIMITED>(3)
        .setAngular0(u0[0])
        .setAngular1(u0[0])
        .setTargetVelocity(driveVelocity)
        .setMax(driveMaxTorque)
        .setMin(-driveMaxTorque);
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

physecs::JointSolverDesc physecs::RevoluteJoint::getSolverDesc(entt::registry &registry, Constraint1DLayout& constraintLayout) {
    constraintLayout.createConstraints();
    constraintLayout.createConstraints<ANGULAR, 2>();
    if (data.driveEnabled) constraintLayout.createConstraints<ANGULAR | LIMITED>();

    const int numConstraints = data.driveEnabled ? 4 : 3;

    return {
        numConstraints,
        &data,
        makeConstraints
    };
}
