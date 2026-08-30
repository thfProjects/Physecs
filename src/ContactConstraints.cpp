#include "ContactConstraints.h"
#include <SolverData.h>

void physecs::ContactConstraints::preSolve(VelocityData* velocities) {
    glm::vec3 velocity0 = asVec3(velocities[b0].velocity);
    glm::vec3 angularVelocity0 = asVec3(velocities[b0].angularVelocity);

    glm::vec3 velocity1 = asVec3(velocities[b1].velocity);
    glm::vec3 angularVelocity1 = asVec3(velocities[b1].angularVelocity);

    for (int i = 0; i < numPoints; ++i) {

        auto& [r0, r1, r0xn, r1xn, targetVelocity, c, totalLambdaN, invEffMassN, distToFrictionAnchor] = contactPointConstraints[i];

        invEffMassN = glm::dot(n0, n0) + glm::dot(n1, n1) + glm::dot(r0xn, r0xn) + glm::dot(r1xn, r1xn);

        totalLambdaN *= 0.5f;
        velocity0 += totalLambdaN * n0;
        angularVelocity0 += totalLambdaN * r0xn;

        velocity1 -= totalLambdaN * n1;
        angularVelocity1 -= totalLambdaN * r1xn;
    }

    // friction
    auto& [r0, r1, t0, t1, r0xt, r1xt, totalLambdaT, totalLambdaTwist, n0, n1, invEffMassT, invEffMassTwist] = frictionConstraints;

    invEffMassT = glm::dot(t0, t0) + glm::dot(t1, t1) + glm::dot(r0xt, r0xt) + glm::dot(r1xt, r1xt);

    invEffMassTwist = glm::dot(n0, n0) + glm::dot(n1, n1);

    velocity0 += totalLambdaT * t0;
    angularVelocity0 += totalLambdaT * r0xt;

    velocity1 -= totalLambdaT * t1;
    angularVelocity1 -= totalLambdaT * r1xt;

    angularVelocity0 += totalLambdaTwist * n0;
    angularVelocity1 -= totalLambdaTwist * n1;

    if (b0 >= 0) {
        velocities[b0].velocity = fromVec3(velocity0);
        velocities[b0].angularVelocity = fromVec3(angularVelocity0);
    }

    if (b1 >= 0) {
        velocities[b1].velocity = fromVec3(velocity1);
        velocities[b1].angularVelocity = fromVec3(angularVelocity1);
    }
}

void physecs::ContactConstraints::solve(VelocityData* velocities, bool useBias, float timeStep) {
    glm::vec3 velocity0 = asVec3(velocities[b0].velocity);
    glm::vec3 angularVelocity0 = asVec3(velocities[b0].angularVelocity);

    glm::vec3 velocity1 = asVec3(velocities[b1].velocity);
    glm::vec3 angularVelocity1 = asVec3(velocities[b1].angularVelocity);

    float totalNImpulse = 0.f;
    float rEffTimesN = 0.f;

    for (int i = 0; i < numPoints; ++i) {

        auto& [r0, r1, r0xn, r1xn, targetVelocity, c, totalLambdaN, invEffMassN, distToFrictionAnchor] = contactPointConstraints[i];

        if (!invEffMassN) continue;

        float relativeVelocity = glm::dot(-n0, velocity0) + glm::dot(-r0xn, angularVelocity0) + glm::dot(n1, velocity1) + glm::dot(r1xn, angularVelocity1);

        float effMass = 1.f / invEffMassN;
        float lambda;
        if (isSoft) {
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

        totalNImpulse += totalLambdaN;
        rEffTimesN += distToFrictionAnchor * totalLambdaN;

        velocity0 += lambda * n0;
        angularVelocity0 += lambda * r0xn;

        velocity1 -= lambda * n1;
        angularVelocity1 -= lambda * r1xn;
    }

    //friction
    {
        auto& [r0, r1, t0, t1, r0xt, r1xt, totalLambdaT, totalLambdaTwist, n0, n1, invEffMassT, invEffMassTwist] = frictionConstraints;

        if (invEffMassT) {
            float relativeVelocity = glm::dot(-t0, velocity0) + glm::dot(-r0xt, angularVelocity0) + glm::dot(t1, velocity1) + glm::dot(r1xt, angularVelocity1);

            float lambda = relativeVelocity / invEffMassT;

            float frictionLimit = friction * totalNImpulse;
            float prevLambda = totalLambdaT;
            totalLambdaT += lambda;
            totalLambdaT = glm::clamp(totalLambdaT, frictionLimit, -frictionLimit);
            lambda = totalLambdaT - prevLambda;

            velocity0 += lambda * t0;
            angularVelocity0 += lambda * r0xt;

            velocity1 -= lambda * t1;
            angularVelocity1 -= lambda * r1xt;
        }

        if (invEffMassTwist) {
            float relativeVelocity = glm::dot(n1, angularVelocity1) - glm::dot(n0, angularVelocity0);

            float lambda = relativeVelocity / invEffMassTwist;

            float frictionLimit = 0.5f * friction * rEffTimesN;
            float prevLambda = totalLambdaTwist;
            totalLambdaTwist += lambda;
            totalLambdaTwist = glm::clamp(totalLambdaTwist, frictionLimit, -frictionLimit);
            lambda = totalLambdaTwist - prevLambda;

            angularVelocity0 += lambda * n0;

            angularVelocity1 -= lambda * n1;
        }
    }

    velocities[b0].velocity = fromVec3(velocity0);
    velocities[b0].angularVelocity = fromVec3(angularVelocity0);

    velocities[b1].velocity = fromVec3(velocity1);
    velocities[b1].angularVelocity = fromVec3(angularVelocity1);
}
