#include "UniversalJoint.h"

#include <JointsUtil.h>

#include "Constraint1D.h"
#include "Constraint1DContainer.h"

void physecs::UniversalJoint::makeConstraints(JointWorldSpaceData &worldSpaceData, void* /*additionalData*/, Constraint1DWriter& constraints) {
    auto& [p0, p1, r0, r1, u0, u1] = worldSpaceData;

    createPointToPointConstraint(p0, p1, r0, r1, constraints);

    const float d22 = glm::dot(u0[2], u1[2]);
    const glm::vec3 u12xu02 = glm::cross(u1[2], u0[2]);

    constraints.next<ANGULAR>()
    .setAngular0(u12xu02)
    .setAngular1(u12xu02)
    .setC(d22);
}

physecs::JointSolverDesc physecs::UniversalJoint::getSolverDesc(entt::registry &registry, Constraint1DLayout& constraintLayout) {
    constraintLayout.createConstraints<NONE, 3>();
    constraintLayout.createConstraints<ANGULAR>();
    return {
        nullptr,
        makeConstraints
    };
}
