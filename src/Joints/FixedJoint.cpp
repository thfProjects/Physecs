#include "FixedJoint.h"
#include "Constraint1D.h"

void physecs::FixedJoint::makeConstraints(JointWorldSpaceData& worldSpaceData, void* /*additionalData*/, Constraint1DView* constraints) {
    auto& [p0, p1, r0, r1, u0, u1] = worldSpaceData;

    glm::vec3 d = p1 - p0;

    float cn = glm::dot(d, d);
    glm::vec3 r0xd = glm::cross(r0, d);
    glm::vec3 r1xd = glm::cross(r1, d);

    constraints[0]
    .setLinear(d)
    .setAngular0(r0xd)
    .setAngular1(r1xd)
    .setC(cn);

    float d01 = glm::dot(u0[0], u1[1]);
    glm::vec3 u11xu00 = glm::cross(u1[1], u0[0]);

    constraints[1]
    .setAngular0(u11xu00)
    .setAngular1(u11xu00)
    .setC(d01)
    .setFlags(Constraint1D::ANGULAR);

    float d02 = glm::dot(u0[0], u1[2]);
    glm::vec3 u12xu00 = glm::cross(u1[2], u0[0]);

    constraints[2]
    .setAngular0(u12xu00)
    .setAngular1(u12xu00)
    .setC(d02)
    .setFlags(Constraint1D::ANGULAR);

    float d12 = glm::dot(u0[1], u1[2]);
    glm::vec3 u12xu01 = glm::cross(u1[2], u0[1]);

    constraints[3]
    .setAngular0(u12xu01)
    .setAngular1(u12xu01)
    .setC(d12)
    .setFlags(Constraint1D::ANGULAR);
}

physecs::JointSolverData physecs::FixedJoint::getSolverData(entt::registry &registry) {
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
