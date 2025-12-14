#include "PrismaticJoint.h"
#include "Constraint1D.h"
#include "Constraint1DContainer.h"
#include "Transform.h"

void physecs::PrismaticJoint::makeConstraints(JointWorldSpaceData &worldSpaceData, void *additionalData, Constraint1DView* constraints) {
    auto& [p0, p1, r0, r1, u0, u1] = worldSpaceData;
    auto& [upperLimit, lowerLimit, makeUpperLimit, makeLowerLimit, driveEnabled, targetPosition, driveStiffness, driveDamping] = *static_cast<PrismaticJointData*>(additionalData);

    glm::vec3 d = p1 - p0;

    //translation
    const float dy = glm::dot(d, u0[1]);
    const glm::vec3 r0xy = glm::cross(r0, u0[1]);
    const glm::vec3 r1xy = glm::cross(r1, u0[1]);

    constraints[0]
    .setLinear(u0[1])
    .setAngular0(r0xy)
    .setAngular1(r1xy)
    .setC(dy);

    const float dz = glm::dot(d, u0[2]);
    const glm::vec3 r0xz = glm::cross(r0, u0[2]);
    const glm::vec3 r1xz = glm::cross(r1, u0[2]);

    constraints[1]
   .setLinear(u0[2])
   .setAngular0(r0xz)
   .setAngular1(r1xz)
   .setC(dz);

    //rotation
    const float d01 = glm::dot(u0[0], u1[1]);
    const glm::vec3 u11xu00 = glm::cross(u1[1], u0[0]);

    constraints[2]
   .setAngular0(u11xu00)
   .setAngular1(u11xu00)
   .setC(d01)
   .setFlags(Constraint1D::ANGULAR);

    const float d02 = glm::dot(u0[0], u1[2]);
    const glm::vec3 u12xu00 = glm::cross(u1[2], u0[0]);

    constraints[3]
   .setAngular0(u12xu00)
   .setAngular1(u12xu00)
   .setC(d02)
   .setFlags(Constraint1D::ANGULAR);

    const float d12 = glm::dot(u0[1], u1[2]);
    const glm::vec3 u12xu01 = glm::cross(u1[2], u0[1]);

    constraints[4]
    .setAngular0(u12xu01)
    .setAngular1(u12xu01)
    .setC(d12)
    .setFlags(Constraint1D::ANGULAR);

    const float dx = glm::dot(d, u0[0]);
    const glm::vec3 r0xx = glm::cross(r0, u0[0]);
    const glm::vec3 r1xx = glm::cross(r1, u0[0]);

    //limits
    int index = 5;
    if (makeUpperLimit) {
        constraints[index]
        .setLinear(u0[0])
        .setAngular0(r0xx)
        .setAngular1(r1xx)
        .setC(dx - upperLimit)
        .setMin(0)
        .setFlags(Constraint1D::LIMITED);

        index++;
    }
    else if (makeLowerLimit) {
        constraints[index]
        .setLinear(u0[0])
        .setAngular0(r0xx)
        .setAngular1(r1xx)
        .setC(dx - lowerLimit)
        .setMax(0)
        .setFlags(Constraint1D::LIMITED);

        index++;
    }

    //drive
    if (driveEnabled) {
        constraints[index]
       .setLinear(u0[0])
       .setAngular0(r0xx)
       .setAngular1(r1xx)
       .setC(dx - targetPosition)
       .setFlags(Constraint1D::SOFT)
       .setFrequency(driveStiffness)
       .setDampingRatio(driveDamping);
    }
}

void physecs::PrismaticJoint::setUpperLimit(float upperLimit) {
    data.upperLimit = upperLimit;
}

void physecs::PrismaticJoint::setLowerLimit(float lowerLimit) {
    data.lowerLimit = lowerLimit;
}

void physecs::PrismaticJoint::setDriveEnabled(bool driveEnabled) {
    data.driveEnabled = driveEnabled;
}

void physecs::PrismaticJoint::setTargetPosition(float targetPosition) {
    data.targetPosition = targetPosition;
}

void physecs::PrismaticJoint::setDriveStiffness(float driveStiffness) {
    data.driveStiffness = driveStiffness;
}

void physecs::PrismaticJoint::setDriveDamping(float driveDamping) {
    data.driveDamping = driveDamping;
}

physecs::JointSolverDesc physecs::PrismaticJoint::getSolverDesc(entt::registry &registry) {
    const auto& transform0 = registry.get<TransformComponent>(entity0);
    const auto& transform1 = registry.get<TransformComponent>(entity1);

    const glm::vec3 p0 = transform0.position + transform0.orientation * anchor0Pos;
    const glm::vec3 p1 = transform1.position + transform1.orientation * anchor1Pos;

    const glm::vec3 d = p1 - p0;

    const glm::vec3 u00 = transform0.orientation * anchor0Or * glm::vec3(1, 0, 0);

    const float dx = glm::dot(d, u00);

    if (dx > data.upperLimit) {
        data.makeUpperLimit = true;
        data.makeLowerLimit = false;
    }
    else if (dx < data.lowerLimit) {
        data.makeUpperLimit = false;
        data.makeLowerLimit = true;
    }
    else {
        data.makeUpperLimit = false;
        data.makeLowerLimit = false;
    }

    const int numConstraints = 5 + (data.makeUpperLimit || data.makeLowerLimit) + data.driveEnabled;

    return {
        numConstraints,
        &data,
        makeConstraints
    };
}
