#include "ContactConstraints.h"
#include <SolverData.h>

void physecs::ContactConstraints::preSolve(const MassData* masses, VelocityData* velocities) {
    glm::vec3 velocity0 = asVec3(velocities[b0].velocity);
    glm::vec3 angularVelocity0 = asVec3(velocities[b0].angularVelocity);
    invMass0 = masses[b0].invMass;
    const glm::mat3& invInertiaTensor0 = masses[b0].invInertiaTensor;

    glm::vec3 velocity1 = asVec3(velocities[b1].velocity);
    glm::vec3 angularVelocity1 = asVec3(velocities[b1].angularVelocity);
    invMass1 = masses[b1].invMass;
    const glm::mat3& invInertiaTensor1 = masses[b1].invInertiaTensor;

    for (int i = 0; i < numPoints; ++i) {

        auto& [r0, r1, r0xn, r1xn, targetVelocity, c, totalLambdaN, r0xnt, r1xnt, invEffMassN, distToFrictionAnchor] = contactPointConstraints[i];

        r0xnt = invInertiaTensor0 * r0xn;
        r1xnt = invInertiaTensor1 * r1xn;

        invEffMassN = glm::dot(n, n) * (invMass0 + invMass1) + glm::dot(r0xn, r0xnt) + glm::dot(r1xn, r1xnt);

        totalLambdaN *= 0.5f;
        velocity0 += totalLambdaN * invMass0 * n;
        angularVelocity0 += totalLambdaN * r0xnt;

        velocity1 -= totalLambdaN * invMass1 * n;
        angularVelocity1 -= totalLambdaN * r1xnt;
    }

    // friction
    auto& [r0, r1, t, r0xt, r1xt, totalLambdaT, totalLambdaTwist, r0xtt, r1xtt, n0t, n1t, invEffMassT, invEffMassTwist] = frictionConstraints;

    r0xtt = invInertiaTensor0 * r0xt;
    r1xtt = invInertiaTensor1 * r1xt;

    invEffMassT = glm::dot(t, t) * (invMass0 + invMass1) + glm::dot(r0xt, r0xtt) + glm::dot(r1xt, r1xtt);

    n0t = invInertiaTensor0 * n;
    n1t = invInertiaTensor1 * n;

    invEffMassTwist = glm::dot(n, n0t + n1t);

    velocity0 += totalLambdaT * invMass0 * t;
    angularVelocity0 += totalLambdaT * r0xtt;

    velocity1 -= totalLambdaT * invMass1 * t;
    angularVelocity1 -= totalLambdaT * r1xtt;

    angularVelocity0 += totalLambdaTwist * n0t;
    angularVelocity1 -= totalLambdaTwist * n1t;

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

        auto& [r0, r1, r0xn, r1xn, targetVelocity, c, totalLambdaN, r0xnt, r1xnt, invEffMassN, distToFrictionAnchor] = contactPointConstraints[i];

        if (!invEffMassN) continue;

        float relativeVelocity = glm::dot(-n, velocity0) + glm::dot(-r0xn, angularVelocity0) + glm::dot(n, velocity1) + glm::dot(r1xn, angularVelocity1);

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

        velocity0 += lambda * invMass0 * n;
        angularVelocity0 += lambda * r0xnt;

        velocity1 -= lambda * invMass1 * n;
        angularVelocity1 -= lambda * r1xnt;
    }

    //friction
    {
        auto& [r0, r1, t, r0xt, r1xt, totalLambdaT, totalLambdaTwist, r0xtt, r1xtt, n0t, n1t, invEffMassT, invEffMassTwist] = frictionConstraints;

        if (invEffMassT) {
            float relativeVelocity = glm::dot(-t, velocity0) + glm::dot(-r0xt, angularVelocity0) + glm::dot(t, velocity1) + glm::dot(r1xt, angularVelocity1);

            float lambda = relativeVelocity / invEffMassT;

            float frictionLimit = friction * totalNImpulse;
            float prevLambda = totalLambdaT;
            totalLambdaT += lambda;
            totalLambdaT = glm::clamp(totalLambdaT, frictionLimit, -frictionLimit);
            lambda = totalLambdaT - prevLambda;

            velocity0 += lambda * invMass0 * t;
            angularVelocity0 += lambda * r0xtt;

            velocity1 -= lambda * invMass1 * t;
            angularVelocity1 -= lambda * r1xtt;
        }

        if (invEffMassTwist) {
            float relativeVelocity = glm::dot(n, angularVelocity1 - angularVelocity0);

            float lambda = relativeVelocity / invEffMassTwist;

            float frictionLimit = 0.5f * friction * rEffTimesN;
            float prevLambda = totalLambdaTwist;
            totalLambdaTwist += lambda;
            totalLambdaTwist = glm::clamp(totalLambdaTwist, frictionLimit, -frictionLimit);
            lambda = totalLambdaTwist - prevLambda;

            angularVelocity0 += lambda * n0t;

            angularVelocity1 -= lambda * n1t;
        }
    }

    velocities[b0].velocity = fromVec3(velocity0);
    velocities[b0].angularVelocity = fromVec3(angularVelocity0);

    velocities[b1].velocity = fromVec3(velocity1);
    velocities[b1].angularVelocity = fromVec3(angularVelocity1);
}
