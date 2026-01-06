#include "FixedJoint.h"
#include "Constraint1D.h"
#include "Constraint1DContainer.h"
#include "JointsUtil.h"

void physecs::FixedJoint::makeConstraints(JointWorldSpaceData& worldSpaceData, void* /*additionalData*/, Constraint1DWriter& constraints) {
    auto& [p0, p1, r0, r1, u0, u1] = worldSpaceData;

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

    const float d12 = glm::dot(u0[1], u1[2]);
    const glm::vec3 u12xu01 = glm::cross(u1[2], u0[1]);

    constraints.next<ANGULAR>()
    .setAngular0(u12xu01)
    .setAngular1(u12xu01)
    .setC(d12);
}

physecs::JointSolverDesc physecs::FixedJoint::getSolverDesc(entt::registry &registry, Constraint1DLayout& constraintLayout) {
    constraintLayout.createConstraints<NONE, 3>();
    constraintLayout.createConstraints<ANGULAR, 3>();
    return {
        nullptr,
        makeConstraints
    };
}
