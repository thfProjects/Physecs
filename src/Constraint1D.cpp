#include "Constraint1D.h"
#include "Components.h"
#include "Transform.h"

template<int flags>
void physecs::Constraint1D<flags>::preSolve() {
    float invMass0 = 0;
    glm::mat3 invInertiaTensor0(0);
    if (dynamic0 && !dynamic0->isKinematic) {
        invMass0 = dynamic0->invMass;
        invInertiaTensor0 = dynamic0->invInertiaTensorWorld;
    }

    float invMass1 = 0;
    glm::mat3 invInertiaTensor1(0);
    if (dynamic1 && !dynamic1->isKinematic) {
        invMass1 = dynamic1->invMass;
        invInertiaTensor1 = dynamic1->invInertiaTensorWorld;
    }

    angular0t = invInertiaTensor0 * angular0;
    angular1t = invInertiaTensor1 * angular1;

    invEffMass = glm::dot(angular0, angular0t) + glm::dot(angular1, angular1t);
    if constexpr (!(flags & ANGULAR)) {
        invEffMass += glm::dot(linear, linear) * (invMass0 + invMass1);
    }

    if constexpr (flags & SOFT) return;

    //correct position error
    if (c && invEffMass) {
        float lambda = 0.1f * c / invEffMass;
        if constexpr (flags & LIMITED)
            lambda = glm::clamp(lambda, min, max);

        if (dynamic0 && !dynamic0->isKinematic) {
            if constexpr (!(flags & ANGULAR))
                dynamic0->pseudoVelocity += lambda * invMass0 * linear;
            dynamic0->pseudoAngularVelocity += lambda * angular0t;
        }

        if (dynamic1 && !dynamic1->isKinematic) {
            if constexpr (!(flags & ANGULAR))
                dynamic1->pseudoVelocity -= lambda * invMass1 * linear;
            dynamic1->pseudoAngularVelocity -= lambda * angular1t;
        }
    }

    // warm start
    if (glm::abs(c) > 1e-4 || glm::abs(totalLambda) > 10000) return;

    totalLambda = totalLambda * 0.5f;
    if (dynamic0 && !dynamic0->isKinematic) {
        if constexpr (!(flags & ANGULAR))
            dynamic0->velocity += totalLambda * dynamic0->invMass * linear;
        dynamic0->angularVelocity += totalLambda * angular0t;
    }

    if (dynamic1 && !dynamic1->isKinematic) {
        if constexpr (!(flags & ANGULAR))
            dynamic1->velocity -= totalLambda * dynamic1->invMass * linear;
        dynamic1->angularVelocity -= totalLambda * angular1t;
    }
}

template<int flags>
void physecs::Constraint1D<flags>::solve(float timeStep) {
    if (!invEffMass) return;

    glm::vec3 velocity0(0), angularVelocity0(0);
    if (dynamic0 && !dynamic0->isKinematic) {
        velocity0 = dynamic0->velocity;
        angularVelocity0 = dynamic0->angularVelocity;
    }

    glm::vec3 velocity1(0), angularVelocity1(0);
    if (dynamic1 && !dynamic1->isKinematic) {
        velocity1 = dynamic1->velocity;
        angularVelocity1 = dynamic1->angularVelocity;
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

    if (dynamic0 && !dynamic0->isKinematic) {
        if constexpr (!(flags & ANGULAR))
            dynamic0->velocity += lambda * dynamic0->invMass * linear;
        dynamic0->angularVelocity += lambda * angular0t;
    }

    if (dynamic1 && !dynamic1->isKinematic) {
        if constexpr (!(flags & ANGULAR))
            dynamic1->velocity -= lambda * dynamic1->invMass * linear;
        dynamic1->angularVelocity -= lambda * angular1t;
    }
}
