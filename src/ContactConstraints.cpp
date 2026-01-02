#include "ContactConstraints.h"

void physecs::ContactConstraints::preSolve() {
    // for (int i = 0; i < numPoints; ++i) {
    //     auto& [r0, r1, r0xn, r1xn, t, r0xt, r1xt, targetVelocity, bias, totalLambdaN, totalLambdaT] = contactPointConstraints[i];
    //
    //     if (dynamic0 && !dynamic0->isKinematic) {
    //         dynamic0->velocity += totalLambdaN * dynamic0->invMass * n + totalLambdaT * dynamic0->invMass * t;
    //         dynamic0->angularVelocity += totalLambdaN * dynamic0->invInertiaTensorWorld * r0xn + totalLambdaT * dynamic0->invInertiaTensorWorld * r0xt;
    //     }
    //
    //     if (dynamic1 && !dynamic1->isKinematic) {
    //         dynamic1->velocity -= totalLambdaN * dynamic1->invMass * n + totalLambdaT * dynamic1->invMass * t;
    //         dynamic1->angularVelocity -= totalLambdaN * dynamic1->invInertiaTensorWorld * r1xn + totalLambdaT * dynamic1->invInertiaTensorWorld * r1xt;
    //     }
    // }
}

void physecs::ContactConstraints::solve(bool useBias, float timeStep) {
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

    for (int i = 0; i < numPoints; ++i) {

        auto& [r0, r1, r0xn, r1xn, t, r0xt, r1xt, targetVelocity, c, totalLambdaN, totalLambdaT] = contactPointConstraints[i];

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

        float relativeVelocity = glm::dot(-n, velocity0) + glm::dot(-r0xn, angularVelocity0) + glm::dot(n, velocity1) + glm::dot(r1xn, angularVelocity1);

        float invEffMass = glm::dot(n, n) * (invMass0 + invMass1) + glm::dot(r0xn, invInertiaTensor0 * r0xn) + glm::dot(r1xn, invInertiaTensor1 * r1xn);

        float lambda;
        if (isSoft) {
            float angularFreq = 2.f * glm::pi<float>() * frequency;
            float stiffness = angularFreq * angularFreq / invEffMass;
            float damping = 2.f * angularFreq * dampingRatio / invEffMass;
            float gamma = 1.f / (damping + timeStep * stiffness);
            float beta = timeStep * stiffness / (damping + timeStep * stiffness);
            lambda = (relativeVelocity + beta * c / timeStep) / (invEffMass + gamma / timeStep);
        } else {
            lambda = (relativeVelocity - targetVelocity + (useBias ? 0.1f * c / timeStep : 0)) / invEffMass;
        }

        float prevLambda = totalLambdaN;
        totalLambdaN += lambda;
        totalLambdaN = glm::min(totalLambdaN, 0.f);
        lambda = totalLambdaN - prevLambda;

        //if (timeStep) printf("normal force: %f\n", totalLambdaN / timeStep);

        if (dynamic0 && !dynamic0->isKinematic) {
            dynamic0->velocity += lambda * invMass0 * n;
            dynamic0->angularVelocity += lambda * invInertiaTensor0 * r0xn;
        }

        if (dynamic1 && !dynamic1->isKinematic) {
            dynamic1->velocity -= lambda * invMass1 * n;
            dynamic1->angularVelocity -= lambda * invInertiaTensor1 * r1xn;
        }
    }

    //friction
    for (int i = 0; i < numPoints; ++i) {

        auto& [r0, r1, r0xn, r1xn, t, r0xt, r1xt, targetVelocity, bias, totalLambdaN, totalLambdaT] = contactPointConstraints[i];

        float dtt = glm::dot(t, t);
        if (!dtt) continue;

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

        float relativeVelocity = glm::dot(-t, velocity0) + glm::dot(-r0xt, angularVelocity0) + glm::dot(t, velocity1) + glm::dot(r1xt, angularVelocity1);

        float lambda = relativeVelocity / (dtt * (invMass0 + invMass1) + glm::dot(r0xt, invInertiaTensor0 * r0xt) + glm::dot(r1xt, invInertiaTensor1 * r1xt));

        float frictionLimit = friction * totalLambdaN;
        float prevLambda = totalLambdaT;
        totalLambdaT += lambda;
        totalLambdaT = glm::clamp(totalLambdaT, frictionLimit, -frictionLimit);
        lambda = totalLambdaT - prevLambda;

        if (dynamic0 && !dynamic0->isKinematic) {
            dynamic0->velocity += lambda * invMass0 * t;
            dynamic0->angularVelocity += lambda * invInertiaTensor0 * r0xt;
        }

        if (dynamic1 && !dynamic1->isKinematic) {
            dynamic1->velocity -= lambda * invMass1 * t;
            dynamic1->angularVelocity -= lambda * invInertiaTensor1 * r1xt;
        }
    }
}
