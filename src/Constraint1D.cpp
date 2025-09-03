#include "Constraint1D.h"
#include <iostream>

void physecs::Constraint1D::prepare() {
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

    r0xnt = invInertiaTensor0 * r0xn;
    r1xnt = invInertiaTensor1 * r1xn;

    invEffMass = glm::dot(r0xn, r0xnt) + glm::dot(r1xn, r1xnt);
    if (!(flags & ANGULAR)) {
        invEffMass += glm::dot(n, n) * (invMass0 + invMass1);
    }

    //correct position error
    if (flags & SOFT || !c || !invEffMass) return;

    float lambda = 0.1f * c / invEffMass;
    if (flags & LIMITED)
        lambda = glm::clamp(lambda, min, max);

    if (!(flags & ANGULAR)) {
        transform0.position += lambda * invMass0 * n;
        transform1.position -= lambda * invMass1 * n;
    }

    transform0.orientation += 0.5f * glm::quat(0, lambda * r0xnt) * transform0.orientation;
    transform0.orientation = glm::normalize(transform0.orientation);

    transform1.orientation -= 0.5f * glm::quat(0, lambda * r1xnt) * transform1.orientation;
    transform1.orientation = glm::normalize(transform1.orientation);
}

void physecs::Constraint1D::solve(bool useBias, float timeStep) {
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

    float relativeVelocity = glm::dot(-r0xn, angularVelocity0) + glm::dot(r1xn, angularVelocity1);
    if (!(flags & ANGULAR)) {
        relativeVelocity += glm::dot(-n, velocity0) + glm::dot(n, velocity1);
    }

    float lambda;
    if (flags & SOFT) {
        float angularFreq = 2.f * glm::pi<float>() * frequency;
        float stiffness = angularFreq * angularFreq / invEffMass;
        float damping = 2.f * angularFreq * dampingRatio / invEffMass;
        float gamma = 1.f / (damping + timeStep * stiffness);
        float beta = timeStep * stiffness / (damping + timeStep * stiffness);
        lambda = (relativeVelocity + beta * c / timeStep) / (invEffMass + gamma / timeStep);
    } else {
        lambda = (relativeVelocity - targetVelocity + (useBias ? 0.2f * c / timeStep : 0)) / invEffMass;
    }

    if (flags & LIMITED) {
        float prevLambda = totalLambda;
        totalLambda += lambda;
        totalLambda = glm::clamp(totalLambda, min, max);
        lambda = totalLambda - prevLambda;
    }
    else {
        totalLambda += lambda;
    }

    if (dynamic0 && !dynamic0->isKinematic) {
        if (!(flags & ANGULAR))
            dynamic0->velocity += lambda * dynamic0->invMass * n;
        dynamic0->angularVelocity += lambda * r0xnt;
    }

    if (dynamic1 && !dynamic1->isKinematic) {
        if (!(flags & ANGULAR))
            dynamic1->velocity -= lambda * dynamic1->invMass * n;
        dynamic1->angularVelocity -= lambda * r1xnt;
    }
}

void physecs::Constraint1D::warmStart() {
    totalLambda = totalLambda * 0.5f;
    if (dynamic0 && !dynamic0->isKinematic) {
        if (!(flags & ANGULAR))
            dynamic0->velocity += totalLambda * dynamic0->invMass * n;
        dynamic0->angularVelocity += totalLambda * r0xnt;
    }

    if (dynamic1 && !dynamic1->isKinematic) {
        if (!(flags & ANGULAR))
            dynamic1->velocity -= totalLambda * dynamic1->invMass * n;
        dynamic1->angularVelocity -= totalLambda * r1xnt;
    }
}
