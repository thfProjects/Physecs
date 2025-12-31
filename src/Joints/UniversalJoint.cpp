#include "UniversalJoint.h"
#include "Constraint1D.h"
#include "Constraint1DContainer.h"

void physecs::UniversalJoint::makeConstraints(JointWorldSpaceData &worldSpaceData, void* /*additionalData*/, Constraint1DWriter& constraints) {
    auto& [p0, p1, r0, r1, u0, u1] = worldSpaceData;

    const glm::vec3 d = p1 - p0;

    const float cn = glm::dot(d, d);
    const glm::vec3 r0xd = glm::cross(r0, d);
    const glm::vec3 r1xd = glm::cross(r1, d);

    constraints.at(0)
    .setLinear(d)
    .setAngular0(r0xd)
    .setAngular1(r1xd)
    .setC(cn);

    const float d22 = glm::dot(u0[2], u1[2]);
    const glm::vec3 u12xu02 = glm::cross(u1[2], u0[2]);

    constraints.at<ANGULAR>(1)
    .setAngular0(u12xu02)
    .setAngular1(u12xu02)
    .setC(d22);
}

physecs::JointSolverDesc physecs::UniversalJoint::getSolverDesc(entt::registry &registry, Constraint1DLayout& constraintLayout) {
    constraintLayout.createConstraints();
    constraintLayout.createConstraints<ANGULAR>();
    return {
        numConstraints,
        nullptr,
        makeConstraints
    };
}
