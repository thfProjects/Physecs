#include "GearJoint.h"
#include "Constraint1D.h"

static float angleDiff(float angle0, float angle1) {
    const float diff = fmodf(angle1 - angle0 + glm::pi<float>(), glm::two_pi<float>()) - glm::pi<float>();
    return diff < -glm::pi<float>() ? diff + glm::two_pi<float>() : diff;
}

void physecs::GearJoint::makeConstraints(JointWorldSpaceData& worldSpaceData, void* additionalData, Constraint1DView* constraints) {
    auto& [p0, p1, r0, r1, u0, u1] = worldSpaceData;
    auto& [gearRatio, persistentAngle0, persistentAngle1, virtualAngle0, virtualAngle1, isInitialized] = *static_cast<GearJointData*>(additionalData);

    float angle0, angle1;
    {
        glm::vec3 p1Proj = p1 + glm::dot(p0 - p1, u0[0]) * u0[0];
        glm::vec3 dir = glm::normalize(p0 - p1Proj);
        glm::vec3 n = glm::cross(u0[0], dir);
        glm::mat3 m(u0[0], -n, dir);
        glm::mat3 u0t = glm::transpose(m) * u0;
        angle0 = glm::atan(u0t[1][2], u0t[2][2]);
    }
    {
        glm::vec3 p0Proj = p0 + glm::dot(p1 - p0, u1[0]) * u1[0];
        glm::vec3 dir = glm::normalize(p0Proj - p1);
        glm::vec3 n = glm::cross(u1[0], dir);
        glm::mat3 m(u1[0], -n, dir);
        glm::mat3 u1t = glm::transpose(u1) * m;
        angle1 = glm::atan(u1t[1][2], u1t[2][2]);
    }

    if (!isInitialized) {
        persistentAngle0 = angle0;
        persistentAngle1 = angle1;
        isInitialized = true;
    }

    const float travelThisFrame0 = angleDiff(angle0, persistentAngle0);
    const float travelThisFrame1 = angleDiff(angle1, persistentAngle1);
    virtualAngle0 += travelThisFrame0;
    virtualAngle1 += travelThisFrame1;

    persistentAngle0 = angle0;
    persistentAngle1 = angle1;

    constraints[0]
    .setAngular0(u0[0] * gearRatio)
    .setAngular1(-u1[0])
    .setC(virtualAngle0 * gearRatio - virtualAngle1)
    .setFlags(Constraint1D::ANGULAR);
}

void physecs::GearJoint::setGearRatio(float gearRatio) {
    data.gearRatio = gearRatio;
}

physecs::JointSolverData physecs::GearJoint::getSolverData(entt::registry &registry) {
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
        &data,
        makeConstraints
    };
}
