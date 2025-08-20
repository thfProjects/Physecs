#include "SphericalJoint.h"
#include "Constraint1D.h"

void physecs::SphericalJoint::makeConstraints(JointWorldSpaceData &worldSpaceData, void* /*additionalData*/, Constraint1D *constraints) {
    auto& [p0, p1, r0, r1, u0, u1] = worldSpaceData;

    glm::vec3 d = p1 - p0;

    float cn = glm::dot(d, d);
    glm::vec3 r0xd = glm::cross(r0, d);
    glm::vec3 r1xd = glm::cross(r1, d);

    constraints[0].n = d;
    constraints[0].r0xn = r0xd;
    constraints[0].r1xn = r1xd;
    constraints[0].c = cn;
}

physecs::JointSolverData physecs::SphericalJoint::getSolverData(entt::registry &registry) {
    return {
        registry.get<TransformComponent>(entity0),
        registry.get<TransformComponent>(entity1),
        registry.try_get<RigidBodyDynamicComponent>(entity0),
        registry.try_get<RigidBodyDynamicComponent>(entity1),
        anchor0Pos,
        anchor0Or,
        anchor1Pos,
        anchor1Or,
        numConstraints,
        nullptr,
        makeConstraints
    };
}
