#include "PrismaticJoint.h"
#include <limits>
#include "Constraint1D.h"
#include "Constraint1DContainer.h"
#include "Transform.h"

void physecs::PrismaticJoint::makeConstraints(const JointWorldSpaceData &worldSpaceData, void *additionalData, Constraint1DDescriptor* constraintRows) {
    auto& [p0, p1, r0, r1, u0, u1] = worldSpaceData;
    auto& [upperLimit, lowerLimit, makeUpperLimit, makeLowerLimit, driveEnabled, targetPosition, driveStiffness, driveDamping] = *static_cast<PrismaticJointDef::Data*>(additionalData);

    glm::vec3 d = p1 - p0;

    // prevents creating net torque on body pair, forces must be applied on same point
    const glm::vec3 a0 = r0 + d;

    //rotation
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

    const float d12 = glm::dot(u0[1], u1[2]);
    const glm::vec3 u12xu01 = glm::cross(u1[2], u0[1]);

    constraintRows[2].angular0 = u12xu01;
    constraintRows[2].angular1 = u12xu01;
    constraintRows[2].geometricError = d12;

    //translation
    const float dy = glm::dot(d, u0[1]);
    const glm::vec3 r0xy = glm::cross(a0, u0[1]);
    const glm::vec3 r1xy = glm::cross(r1, u0[1]);

    constraintRows[3].linear0 = u0[1];
    constraintRows[3].linear1 = u0[1];
    constraintRows[3].angular0 = r0xy;
    constraintRows[3].angular1 = r1xy;
    constraintRows[3].geometricError = dy;

    const float dz = glm::dot(d, u0[2]);
    const glm::vec3 r0xz = glm::cross(a0, u0[2]);
    const glm::vec3 r1xz = glm::cross(r1, u0[2]);

    constraintRows[4].linear0 = u0[2];
    constraintRows[4].linear1 = u0[2];
    constraintRows[4].angular0 = r0xz;
    constraintRows[4].angular1 = r1xz;
    constraintRows[4].geometricError = dz;

    const float dx = glm::dot(d, u0[0]);
    const glm::vec3 r0xx = glm::cross(a0, u0[0]);
    const glm::vec3 r1xx = glm::cross(r1, u0[0]);

    // the gated blocks are packed densely, so they are indexed at runtime
    int row = 5;

    //limits
    if (makeUpperLimit) {
        constraintRows[row].linear0 = u0[0];
        constraintRows[row].linear1 = u0[0];
        constraintRows[row].angular0 = r0xx;
        constraintRows[row].angular1 = r1xx;
        constraintRows[row].geometricError = dx - upperLimit;
        constraintRows[row].minForce = 0;
        constraintRows[row].maxForce = std::numeric_limits<float>::max();
        ++row;
    }
    else if (makeLowerLimit) {
        constraintRows[row].linear0 = u0[0];
        constraintRows[row].linear1 = u0[0];
        constraintRows[row].angular0 = r0xx;
        constraintRows[row].angular1 = r1xx;
        constraintRows[row].geometricError = dx - lowerLimit;
        constraintRows[row].minForce = std::numeric_limits<float>::lowest();
        constraintRows[row].maxForce = 0;
        ++row;
    }

    //drive
    if (driveEnabled) {
        constraintRows[row].linear0 = u0[0];
        constraintRows[row].linear1 = u0[0];
        constraintRows[row].angular0 = r0xx;
        constraintRows[row].angular1 = r1xx;
        constraintRows[row].geometricError = dx - targetPosition;
        constraintRows[row].stiffness = driveStiffness;
        constraintRows[row].damping = driveDamping;
        ++row;
    }
}

void physecs::PrismaticJoint::prepare(const entt::registry &registry) {
    const auto& transform0 = registry.get<TransformComponent>(entity0);
    const auto& transform1 = registry.get<TransformComponent>(entity1);

    const glm::vec3 p0 = transform0.position + transform0.orientation * anchor0Pos;
    const glm::vec3 p1 = transform1.position + transform1.orientation * anchor1Pos;

    const glm::vec3 d = p1 - p0;

    const glm::vec3 u00 = transform0.orientation * anchor0Or * glm::vec3(1, 0, 0);

    const float dx = glm::dot(d, u00);

    data.makeUpperLimit = dx > data.upperLimit;
    data.makeLowerLimit = !data.makeUpperLimit && dx < data.lowerLimit;
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

namespace physecs {
    PHYSECS_DEFINE_JOINT_IMPL(PrismaticJoint);
}
