#include "GearJoint.h"
#include "Constraint1D.h"
#include "Constraint1DContainer.h"

static float angleDiff(float angle0, float angle1) {
    const float diff = fmodf(angle1 - angle0 + glm::pi<float>(), glm::two_pi<float>()) - glm::pi<float>();
    return diff < -glm::pi<float>() ? diff + glm::two_pi<float>() : diff;
}

void physecs::GearJoint::makeConstraints(const JointWorldSpaceData& worldSpaceData, void* additionalData, Constraint1DWriter& constraints) {
    auto& [p0, p1, r0, r1, u0, u1] = worldSpaceData;
    auto& [gearRatio, persistentAngle0, persistentAngle1, slip, isInitialized] = *static_cast<GearJointData*>(additionalData);

    constexpr float epsilon = 1e-4f;

    auto inert = [&] {
        isInitialized = false;
        constraints.next<>()
        .setLinear(glm::vec3(0))
        .setAngular0(glm::vec3(0))
        .setAngular1(glm::vec3(0))
        .setC(0);
    };

    glm::vec3 d = p1 - p0;
    glm::vec3 k = u0[0] + gearRatio * u1[0]; // line along which gears touch
    glm::vec3 q = gearRatio * (glm::dot(k, d) * u1[0] + glm::dot(k, u1[0]) * d) / length2(k) + p0; // point on k closest to p0
    float s0 = glm::dot(p0 - q, u0[0]) / glm::dot(k, u0[0]); // scalar such that q + s0 * k - p0 is orthogonal to the axis of gear 0
    float s1 = glm::dot(p1 - q, u1[0]) / glm::dot(k, u1[0]); // scalar such that q + s1 * k - p1 is orthogonal to the axis of gear 1
    float s = 0.5f * (s0 + s1);
    glm::vec3 meshPoint = q + s * k;

    glm::vec3 arm0 = meshPoint - p0;
    glm::vec3 arm1 = meshPoint - p1;

    const glm::vec3 tRaw = glm::cross(u0[0], arm0);
    const float rho0 = glm::length(tRaw); // pitch radius
    if (rho0 < epsilon) { inert(); return; }
    const glm::vec3 t = tRaw / rho0;
    const float rho1 = glm::dot(t, glm::cross(u1[0], arm1));
    if (glm::abs(rho1) < epsilon) { inert(); return; }

    const float angle0 = glm::atan(glm::dot(u0[2], arm0), glm::dot(u0[1], arm0));
    const float angle1 = glm::atan(glm::dot(u1[2], arm1), glm::dot(u1[1], arm1));

    if (!isInitialized) {
        persistentAngle0 = angle0;
        persistentAngle1 = angle1;
        isInitialized = true;
    }

    const float travelThisFrame0 = angleDiff(angle0, persistentAngle0);
    const float travelThisFrame1 = angleDiff(angle1, persistentAngle1);

    persistentAngle0 = angle0;
    persistentAngle1 = angle1;

    slip += travelThisFrame1 * rho1 - travelThisFrame0 * rho0;

    constraints.next<>()
    .setLinear(t)
    .setAngular0(glm::cross(r0 + arm0, t))
    .setAngular1(glm::cross(r1 + arm1, t))
    .setC(slip);
}

void physecs::GearJoint::setGearRatio(float gearRatio) {
    data.gearRatio = gearRatio;
}

physecs::JointSolverDesc physecs::GearJoint::getSolverDesc(entt::registry &registry, Constraint1DLayout& constraintLayout) {
    constraintLayout.createConstraints<NONE>(&impulseCache.lambda);
    return {
        &data,
        makeConstraints
    };
}

void physecs::GearJoint::storeAccumulatedImpulses(Constraint1DReader& constraints) {
    impulseCache.lambda = constraints.nextTotalLambda<NONE>();
}
