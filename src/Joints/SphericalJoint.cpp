#include "SphericalJoint.h"
#include "Constraint1D.h"
#include "Constraint1DContainer.h"

void physecs::SphericalJoint::makeConstraints(JointWorldSpaceData &worldSpaceData, void* /*additionalData*/, Constraint1DWriter& constraints) {
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
}

physecs::JointSolverDesc physecs::SphericalJoint::getSolverDesc(entt::registry &registry, Constraint1DLayout& constraintLayout) {
    constraintLayout.createConstraints();
    return {
        numConstraints,
        nullptr,
        makeConstraints
    };
}
