#include "ContactConstraints.h"
#include <SolverData.h>

void physecs::ContactConstraints::preSolve(const MassData* masses) {
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

    for (int i = 0; i < numPoints; ++i) {

        auto& [r0, r1, r0xn, r1xn, t, r0xt, r1xt, targetVelocity, c, totalLambdaN, totalLambdaT, r0xnt, r1xnt, r0xtt, r1xtt, invEffMassN, invEffMassT] = contactPointConstraints[i];

        r0xnt = invInertiaTensor0 * r0xn;
        r1xnt = invInertiaTensor1 * r1xn;
        r0xtt = invInertiaTensor0 * r0xt;
        r1xtt = invInertiaTensor1 * r1xt;

        invEffMassN = glm::dot(n, n) * (invMass0 + invMass1) + glm::dot(r0xn, r0xnt) + glm::dot(r1xn, r1xnt);
        invEffMassT = glm::dot(t, t) * (invMass0 + invMass1) + glm::dot(r0xt, r0xtt) + glm::dot(r1xt, r1xtt);
    }
}

void physecs::ContactConstraints::solve(VelocityData* velocities, bool useBias, float timeStep) {
    for (int i = 0; i < numPoints; ++i) {

        auto& [r0, r1, r0xn, r1xn, t, r0xt, r1xt, targetVelocity, c, totalLambdaN, totalLambdaT, r0xnt, r1xnt, r0xtt, r1xtt, invEffMassN, invEffMassT] = contactPointConstraints[i];

        if (!invEffMassN) continue;

        glm::vec3 velocity0(0), angularVelocity0(0);
        if (b0 >= 0) {
            velocity0 = velocities[b0].velocity;
            angularVelocity0 = velocities[b0].angularVelocity;
        }

        glm::vec3 velocity1(0), angularVelocity1(0);
        if (b1 >= 0) {
            velocity1 = velocities[b1].velocity;
            angularVelocity1 = velocities[b1].angularVelocity;
        }

        float relativeVelocity = glm::dot(-n, velocity0) + glm::dot(-r0xn, angularVelocity0) + glm::dot(n, velocity1) + glm::dot(r1xn, angularVelocity1);

        float effMass = 1.f / invEffMassN;
        float lambda;
        if (isSoft) {
            float angularFreq = 2.f * glm::pi<float>() * frequency;
            float stiffness = angularFreq * angularFreq * effMass;
            float damping = 2.f * angularFreq * dampingRatio * effMass;
            float gamma = 1.f / (damping + timeStep * stiffness);
            float beta = timeStep * stiffness / (damping + timeStep * stiffness);
            lambda = (relativeVelocity + beta * c / timeStep) / (invEffMassN + gamma / timeStep);
        } else {
            lambda = (relativeVelocity - targetVelocity + (useBias ? 0.1f * c / timeStep : 0)) * effMass;
        }

        float prevLambda = totalLambdaN;
        totalLambdaN += lambda;
        totalLambdaN = glm::min(totalLambdaN, 0.f);
        lambda = totalLambdaN - prevLambda;

        //if (timeStep) printf("normal force: %f\n", totalLambdaN / timeStep);

        if (b0 >= 0) {
            velocities[b0].velocity += lambda * dynamic0->invMass * n;
            velocities[b0].angularVelocity += lambda * r0xnt;
        }

        if (b1 >= 0) {
            velocities[b1].velocity -= lambda * dynamic1->invMass * n;
            velocities[b1].angularVelocity -= lambda * r1xnt;
        }
    }

    //friction
    for (int i = 0; i < numPoints; ++i) {

        auto& [r0, r1, r0xn, r1xn, t, r0xt, r1xt, targetVelocity, c, totalLambdaN, totalLambdaT, r0xnt, r1xnt, r0xtt, r1xtt, invEffMassN, invEffMassT] = contactPointConstraints[i];

        if (!invEffMassT) continue;

        glm::vec3 velocity0(0), angularVelocity0(0);
        if (b0 >= 0) {
            velocity0 = velocities[b0].velocity;
            angularVelocity0 = velocities[b0].angularVelocity;
        }

        glm::vec3 velocity1(0), angularVelocity1(0);
        if (b1 >= 0) {
            velocity1 = velocities[b1].velocity;
            angularVelocity1 = velocities[b1].angularVelocity;
        }

        float relativeVelocity = glm::dot(-t, velocity0) + glm::dot(-r0xt, angularVelocity0) + glm::dot(t, velocity1) + glm::dot(r1xt, angularVelocity1);

        float lambda = relativeVelocity / invEffMassT;

        float frictionLimit = friction * totalLambdaN;
        float prevLambda = totalLambdaT;
        totalLambdaT += lambda;
        totalLambdaT = glm::clamp(totalLambdaT, frictionLimit, -frictionLimit);
        lambda = totalLambdaT - prevLambda;

        if (b0 >= 0) {
            velocities[b0].velocity += lambda * dynamic0->invMass * t;
            velocities[b0].angularVelocity += lambda * r0xtt;
        }

        if (b1 >= 0) {
            velocities[b1].velocity -= lambda * dynamic1->invMass * t;
            velocities[b1].angularVelocity -= lambda * r1xtt;
        }
    }
}
