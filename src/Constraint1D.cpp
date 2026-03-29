#include "Constraint1D.h"
#include "SolverData.h"
#include <glm/ext/scalar_constants.hpp>

template<int flags>
void physecs::Constraint1D<flags>::preSolve(const MassData* masses, PseudoVelocityData *pseudoVelocities) {
    float invMass0 = 0;
    glm::mat3 invInertiaTensor0(0);
    if (b0 >= 0) {
        invMass0 = masses[b0].invMass;
        invInertiaTensor0 = masses[b0].invInertiaTensor;
    }

    float invMass1 = 0;
    glm::mat3 invInertiaTensor1(0);
    if (b1 >= 0) {
        invMass1 = masses[b1].invMass;
        invInertiaTensor1 = masses[b1].invInertiaTensor;
    }

    angular0t = invInertiaTensor0 * angular0;
    angular1t = invInertiaTensor1 * angular1;

    invEffMass = glm::dot(angular0, angular0t) + glm::dot(angular1, angular1t);
    if constexpr (!(flags & ANGULAR)) {
        invEffMass += glm::dot(linear, linear) * (invMass0 + invMass1);
        linear0t = invMass0 * linear;
        linear1t = invMass1 * linear;
    }

    if constexpr (flags & SOFT) return;

    if (!c || !invEffMass) return;

    float lambda = c / invEffMass;
    if constexpr (flags & LIMITED)
        lambda = glm::clamp(lambda, min, max);

    if (b0 >= 0) {
        if constexpr (!(flags & ANGULAR))
            pseudoVelocities[b0].pseudoVelocity += lambda * linear0t;
        pseudoVelocities[b0].pseudoAngularVelocity += lambda * angular0t;
        ++pseudoVelocities[b0].constraintCount;
    }

    if (b1 >= 0) {
        if constexpr (!(flags & ANGULAR))
            pseudoVelocities[b1].pseudoVelocity -= lambda * linear1t;
        pseudoVelocities[b1].pseudoAngularVelocity -= lambda * angular1t;
        ++pseudoVelocities[b1].constraintCount;
    }
}

template<int flags>
void physecs::Constraint1D<flags>::solve(VelocityData* velocities, float timeStep, bool warmStart) {
    if (!invEffMass) return;

    glm::vec3 velocity0(0), angularVelocity0(0);
    if (b0 >= 0) {
        if constexpr (!(flags & ANGULAR))
            velocity0 = velocities[b0].velocity;
        angularVelocity0 = velocities[b0].angularVelocity;
    }

    glm::vec3 velocity1(0), angularVelocity1(0);
    if (b1 >= 0) {
        if constexpr (!(flags & ANGULAR))
            velocity1 = velocities[b1].velocity;
        angularVelocity1 = velocities[b1].angularVelocity;
    }

    if constexpr (!(flags & SOFT)) {
        if (warmStart && !(glm::abs(c) > 1e-4 || glm::abs(totalLambda) > 10000)) {
            totalLambda = totalLambda * 0.5f;
            if (b0 >= 0) {
                if constexpr (!(flags & ANGULAR))
                    velocities[b0].velocity += totalLambda * linear0t;
                velocities[b0].angularVelocity += totalLambda * angular0t;
            }

            if (b1 >= 0) {
                if constexpr (!(flags & ANGULAR))
                    velocities[b1].velocity -= totalLambda * linear1t;
                velocities[b1].angularVelocity -= totalLambda * angular1t;
            }
        }
    }

    float relativeVelocity = glm::dot(-angular0, angularVelocity0) + glm::dot(angular1, angularVelocity1);
    if constexpr (!(flags & ANGULAR)) {
        relativeVelocity += glm::dot(-linear, velocity0) + glm::dot(linear, velocity1);
    }

    float lambda;
    if constexpr (flags & SOFT) {
        float angularFreq = 2.f * glm::pi<float>() * frequency;
        float stiffness = angularFreq * angularFreq / invEffMass;
        float damping = 2.f * angularFreq * dampingRatio / invEffMass;
        float gamma = 1.f / (damping + timeStep * stiffness);
        float beta = timeStep * stiffness / (damping + timeStep * stiffness);
        lambda = (relativeVelocity + beta * c / timeStep) / (invEffMass + gamma / timeStep);
    } else {
        lambda = (relativeVelocity - targetVelocity + 0.2f * c / timeStep) / invEffMass;
    }

    if constexpr (flags & LIMITED) {
        float prevLambda = totalLambda;
        totalLambda += lambda;
        totalLambda = glm::clamp(totalLambda, min, max);
        lambda = totalLambda - prevLambda;
    }
    else {
        totalLambda += lambda;
    }

    if (b0 >= 0) {
        if constexpr (!(flags & ANGULAR))
            velocities[b0].velocity += lambda * linear0t;
        velocities[b0].angularVelocity += lambda * angular0t;
    }

    if (b1 >= 0) {
        if constexpr (!(flags & ANGULAR))
            velocities[b1].velocity -= lambda * linear1t;
        velocities[b1].angularVelocity -= lambda * angular1t;
    }
}
