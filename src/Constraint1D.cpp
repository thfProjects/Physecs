#include "Constraint1D.h"
#include "SolverData.h"

template<int flags>
void physecs::Constraint1D<flags>::preSolve(const MassData* masses, VelocityData* velocities, PseudoVelocityData *pseudoVelocities) {
    const float invMass0 = masses[b0].invInertiaTensorAndMass.m128_f32[3];
    const glm::vec3& invInertiaTensor0 = asVec3(masses[b0].invInertiaTensorAndMass);

    const float invMass1 = masses[b1].invInertiaTensorAndMass.m128_f32[3];
    const glm::vec3& invInertiaTensor1 = asVec3(masses[b1].invInertiaTensorAndMass);

    angular0t = invInertiaTensor0 * angular0;
    angular1t = invInertiaTensor1 * angular1;

    invEffMass = glm::dot(angular0, angular0t) + glm::dot(angular1, angular1t);
    if constexpr (!(flags & ANGULAR)) {
        invEffMass += glm::dot(linear, linear) * (invMass0 + invMass1);
        linear0t = invMass0 * linear;
        linear1t = invMass1 * linear;
    }

    if constexpr (flags & SOFT) return;

    // warm start
    if (glm::abs(c) > 1e-4 || glm::abs(totalLambda) > 10000) {
        totalLambda = 0.f;
    }
    else {
        totalLambda = totalLambda * 0.5f;

        if constexpr (!(flags & ANGULAR))
            velocities[b0].velocity += totalLambda * linear0t;
        velocities[b0].angularVelocity += totalLambda * angular0t;

        if constexpr (!(flags & ANGULAR))
            velocities[b1].velocity -= totalLambda * linear1t;
        velocities[b1].angularVelocity -= totalLambda * angular1t;
    }

    if (!c || !invEffMass) return;

    float lambda = c / invEffMass;
    if constexpr (flags & LIMITED)
        lambda = glm::clamp(lambda, min, max);

    if constexpr (!(flags & ANGULAR))
        pseudoVelocities[b0].pseudoVelocity += lambda * linear0t;
    pseudoVelocities[b0].pseudoAngularVelocity += lambda * angular0t;
    ++pseudoVelocities[b0].constraintCount;

    if constexpr (!(flags & ANGULAR))
        pseudoVelocities[b1].pseudoVelocity -= lambda * linear1t;
    pseudoVelocities[b1].pseudoAngularVelocity -= lambda * angular1t;
    ++pseudoVelocities[b1].constraintCount;
}

template<int flags>
void physecs::Constraint1D<flags>::solve(VelocityData* velocities, float timeStep, bool useBias) {
    if (!invEffMass) return;

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
        relativeVelocity += glm::dot(linear, velocity1) - glm::dot(linear, velocity0);
    }

    float lambda;
    if constexpr (flags & SOFT) {
        const float gamma = 1.f / (damping + timeStep * stiffness);
        const float beta = timeStep * stiffness / (damping + timeStep * stiffness);
        lambda = (relativeVelocity + beta * c / timeStep) / (invEffMass + gamma / timeStep);
    } else {
        lambda = (relativeVelocity - targetVelocity + (useBias ? baumgarteBias : 0.f) * c / timeStep) / invEffMass;
    }

    if constexpr (flags & LIMITED) {
        const float prevLambda = totalLambda;
        totalLambda += lambda;
        totalLambda = glm::clamp(totalLambda, min * timeStep, max * timeStep);
        lambda = totalLambda - prevLambda;
    }
    else {
        totalLambda += lambda;
    }

    if constexpr (!(flags & ANGULAR))
        velocities[b0].velocity += lambda * linear0t;
    velocities[b0].angularVelocity += lambda * angular0t;

    if constexpr (!(flags & ANGULAR))
        velocities[b1].velocity -= lambda * linear1t;
    velocities[b1].angularVelocity -= lambda * angular1t;
}
