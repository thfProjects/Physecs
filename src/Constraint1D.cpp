#include "Constraint1D.h"
#include "SolverData.h"
#include "Constraint1DFlags.h"

template<int flags>
__forceinline void physecs::Constraint1D<flags>::preSolve(VelocityData* velocities, PseudoVelocityData *pseudoVelocities, float timeStep) {
    if constexpr (flags & SOFT) {
        const float stiffness = timeStep * springParams.stiffness;
        const float damping = timeStep * springParams.damping;
        springParams.cfm = 1.f / (damping + timeStep * stiffness + 1e-8f);
        springParams.erp = stiffness * springParams.cfm;

        float invEffMass = glm::dot(angular0, angular0) + glm::dot(angular1, angular1);
        if constexpr (!(flags & ANGULAR)) {
            invEffMass += glm::dot(linear0, linear0) + glm::dot(linear1, linear1);
        }

        effMass = 1.f / (invEffMass + springParams.cfm);

        return;
    }

    if constexpr (flags & LIMITED) {
        float invEffMass = glm::dot(angular0, angular0) + glm::dot(angular1, angular1);
        if constexpr (!(flags & ANGULAR)) {
            invEffMass += glm::dot(linear0, linear0) + glm::dot(linear1, linear1);
        }

        effMass = 1.f / (invEffMass + 1e-8f);
        min *= timeStep;
        max *= timeStep;
    }

    // warm start
    if (glm::abs(c) > 1e-4 || glm::abs(totalLambda) > 10000) {
        totalLambda = 0.f;
    }
    else {
        totalLambda = totalLambda * 0.5f;

        if constexpr (!(flags & ANGULAR))
            velocities[b0].velocity += totalLambda * linear0;
        velocities[b0].angularVelocity += totalLambda * angular0;

        if constexpr (!(flags & ANGULAR))
            velocities[b1].velocity -= totalLambda * linear1;
        velocities[b1].angularVelocity -= totalLambda * angular1;
    }

    if (!c) return;

    float lambda = c * effMass;
    if constexpr (flags & LIMITED) {
        const float lo = min < 0.f ? std::numeric_limits<float>::lowest() : 0.f;
        const float hi = max > 0.f ? std::numeric_limits<float>::max() : 0.f;
        lambda = glm::clamp(lambda, lo, hi);
    }

    if constexpr (!(flags & ANGULAR))
        pseudoVelocities[b0].pseudoVelocity += lambda * linear0;
    pseudoVelocities[b0].pseudoAngularVelocity += lambda * angular0;
    ++pseudoVelocities[b0].constraintCount;

    if constexpr (!(flags & ANGULAR))
        pseudoVelocities[b1].pseudoVelocity -= lambda * linear1;
    pseudoVelocities[b1].pseudoAngularVelocity -= lambda * angular1;
    ++pseudoVelocities[b1].constraintCount;
}

template<int flags>
__forceinline void physecs::Constraint1D<flags>::solve(VelocityData* velocities, float baumgarteFactor) {
    glm::vec3 velocity0(0);
    if constexpr (!(flags & ANGULAR))
        velocity0 = asVec3(velocities[b0].velocity);
    const glm::vec3 angularVelocity0 = asVec3(velocities[b0].angularVelocity);

    glm::vec3 velocity1(0);
    if constexpr (!(flags & ANGULAR))
        velocity1 = asVec3(velocities[b1].velocity);
    const glm::vec3 angularVelocity1 = asVec3(velocities[b1].angularVelocity);

    float relativeVelocity = glm::dot(angular1, angularVelocity1) - glm::dot(angular0, angularVelocity0);
    if constexpr (!(flags & ANGULAR)) {
        relativeVelocity += glm::dot(linear1, velocity1) - glm::dot(linear0, velocity0);
    }

    float lambda = (relativeVelocity - targetVelocity + (flags & SOFT ? springParams.erp : baumgarteFactor) * c) * effMass;

    if constexpr (flags & LIMITED) {
        const float prevLambda = totalLambda;
        totalLambda += lambda;
        totalLambda = glm::clamp(totalLambda, min, max);
        lambda = totalLambda - prevLambda;
    }
    else {
        totalLambda += lambda;
    }

    if constexpr (!(flags & ANGULAR))
        velocities[b0].velocity += lambda * linear0;
    velocities[b0].angularVelocity += lambda * angular0;

    if constexpr (!(flags & ANGULAR))
        velocities[b1].velocity -= lambda * linear1;
    velocities[b1].angularVelocity -= lambda * angular1;
}
