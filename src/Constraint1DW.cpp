#include "Constraint1DW.h"
#include <Constraint1D.h>
#include "Components.h"
#include "Transform.h"

void physecs::Constraint1DW::preSolve() {
    const unsigned int flagsW = *reinterpret_cast<unsigned int*>(flags);
    auto softW = flagsW & SOFT_W;
    unsigned char* softWBytes = reinterpret_cast<unsigned char*>(&softW);

    Vec3W position0, position1, velocity0, velocity1, angularVelocity0, angularVelocity1;
    QuatW orientation0, orientation1;
    FloatW warmStartMask;
    for (int i = 0; i < 4; ++i) {
        if (!transform0[i]) break;

        glm::mat3 invInertiaTensor0(0);
        if (dynamic0[i] && !dynamic0[i]->isKinematic) {
            invMass0.m128_f32[i] = dynamic0[i]->invMass;
            invInertiaTensor0 = dynamic0[i]->invInertiaTensorWorld;

            if (!(flags[i] & Constraint1D::ANGULAR))
                velocity0.set(dynamic0[i]->velocity, i);
            angularVelocity0.set(dynamic0[i]->angularVelocity, i);
        }

        glm::mat3 invInertiaTensor1(0);
        if (dynamic1[i] && !dynamic1[i]->isKinematic) {
            invMass1.m128_f32[i] = dynamic1[i]->invMass;
            invInertiaTensor1 = dynamic1[i]->invInertiaTensorWorld;

            if (!(flags[i] & Constraint1D::ANGULAR))
                velocity1.set(dynamic1[i]->velocity, i);
            angularVelocity1.set(dynamic1[i]->angularVelocity, i);
        }

        angular0t.x.m128_f32[i] = invInertiaTensor0[0][0] * angular0.x.m128_f32[i] + invInertiaTensor0[1][0] * angular0.y.m128_f32[i] + invInertiaTensor0[2][0] * angular0.z.m128_f32[i];
        angular0t.y.m128_f32[i] = invInertiaTensor0[0][1] * angular0.x.m128_f32[i] + invInertiaTensor0[1][1] * angular0.y.m128_f32[i] + invInertiaTensor0[2][1] * angular0.z.m128_f32[i];
        angular0t.z.m128_f32[i] = invInertiaTensor0[0][2] * angular0.x.m128_f32[i] + invInertiaTensor0[1][2] * angular0.y.m128_f32[i] + invInertiaTensor0[2][2] * angular0.z.m128_f32[i];

        angular1t.x.m128_f32[i] = invInertiaTensor1[0][0] * angular1.x.m128_f32[i] + invInertiaTensor1[1][0] * angular1.y.m128_f32[i] + invInertiaTensor1[2][0] * angular1.z.m128_f32[i];
        angular1t.y.m128_f32[i] = invInertiaTensor1[0][1] * angular1.x.m128_f32[i] + invInertiaTensor1[1][1] * angular1.y.m128_f32[i] + invInertiaTensor1[2][1] * angular1.z.m128_f32[i];
        angular1t.z.m128_f32[i] = invInertiaTensor1[0][2] * angular1.x.m128_f32[i] + invInertiaTensor1[1][2] * angular1.y.m128_f32[i] + invInertiaTensor1[2][2] * angular1.z.m128_f32[i];

        position0.set(transform0[i]->position, i);
        position1.set(transform1[i]->position, i);
        orientation0.set(transform0[i]->orientation, i);
        orientation1.set(transform1[i]->orientation, i);

        warmStartMask.m128_f32[i] = !softWBytes[i] && glm::abs(c.m128_f32[i]) < 1e-4 && glm::abs(totalLambda.m128_f32[i]) < 10000;
    }

    invEffMass = dotW(angular0, angular0t) + dotW(angular1, angular1t);
    if ((flagsW & ANGULAR_W) != ANGULAR_W) {
        invEffMass += dotW(linear, linear) * (invMass0 + invMass1);
    }

    const auto half = _mm_set1_ps(0.5f);

    // warm start
    if (!isZero(warmStartMask)) {
        auto lambda = totalLambda * half;
        lambda = _mm_blendv_ps(_mm_setzero_ps(), lambda, warmStartMask);

        if ((flagsW & ANGULAR_W) != ANGULAR_W) {
            velocity0 += lambda * invMass0 * linear;
            velocity1 -= lambda * invMass1 * linear;
        }

        angularVelocity0 += lambda * angular0t;
        angularVelocity1 -= lambda * angular1t;

        totalLambda = _mm_blendv_ps(totalLambda, lambda, warmStartMask);
    }

    // position correction (NGS)
    const auto softMask = _mm_castsi128_ps(_mm_cmpeq_epi32(_mm_setzero_si128(), _mm_set_epi32(softWBytes[3], softWBytes[2], softWBytes[1], softWBytes[0])));

    const auto invEffMassMask = _mm_cmpneq_ps(invEffMass, _mm_setzero_ps());

    const auto cMask = _mm_cmpneq_ps(c, _mm_setzero_ps());

    auto ngsMask = _mm_and_ps(cMask, _mm_and_ps(invEffMassMask, softMask));

    if (!isZero(ngsMask)) {
        const auto factor = _mm_set1_ps(0.1f);

        auto lambda = factor * c / invEffMass;
        if (flagsW & LIMITED_W) {
            lambda = _mm_min_ps(_mm_max_ps(lambda, min), max);
        }

        lambda = _mm_blendv_ps(_mm_setzero_ps(), lambda, ngsMask);

        if ((flagsW & ANGULAR_W) != ANGULAR_W) {
            position0 += lambda * invMass0 * linear;
            position1 -= lambda * invMass1 * linear;
        }

        orientation0 += half * QuatW(_mm_setzero_ps(), lambda * angular0t) * orientation0;
        orientation0 = normalize(orientation0);

        orientation1 -= half * QuatW(_mm_setzero_ps(), lambda * angular1t) * orientation1;
        orientation1 = normalize(orientation1);
    }

    for (int i = 0; i < 4; ++i) {
        if (!transform0[i]) break;

        if (ngsMask.m128_i32[i]) {
            position0.get(transform0[i]->position, i);
            position1.get(transform1[i]->position, i);
            orientation0.get(transform0[i]->orientation, i);
            orientation1.get(transform1[i]->orientation, i);
        }

        if (warmStartMask.m128_i32[i]) {
            if (dynamic0[i] && !dynamic0[i]->isKinematic) {
                if (!(flags[i] & Constraint1D::ANGULAR))
                    velocity0.get(dynamic0[i]->velocity, i);
                angularVelocity0.get(dynamic0[i]->angularVelocity, i);
            }

            if (dynamic1[i] && !dynamic1[i]->isKinematic) {
                if (!(flags[i] & Constraint1D::ANGULAR))
                    velocity1.get(dynamic1[i]->velocity, i);
                angularVelocity1.get(dynamic1[i]->angularVelocity, i);
            }
        }
    }
}

void physecs::Constraint1DW::solve(float timeStep) {
    auto invEffMassMask = _mm_cmpneq_ps(invEffMass, _mm_setzero_ps());
    if (isZero(invEffMassMask)) return;

    Vec3W velocity0, velocity1, angularVelocity0, angularVelocity1;
    for (int i = 0; i < 4; ++i) {
        if (dynamic0[i] && !dynamic0[i]->isKinematic) {
            if (!(flags[i] & Constraint1D::ANGULAR))
                velocity0.set(dynamic0[i]->velocity, i);
            angularVelocity0.set(dynamic0[i]->angularVelocity, i);
        }

        if (dynamic1[i] && !dynamic1[i]->isKinematic) {
            if (!(flags[i] & Constraint1D::ANGULAR))
                velocity1.set(dynamic1[i]->velocity, i);
            angularVelocity1.set(dynamic1[i]->angularVelocity, i);
        }
    }

    const unsigned int flagsW = *reinterpret_cast<unsigned int*>(flags);

    auto relativeVelocityW = dotW(angular1, angularVelocity1) - dotW(angular0, angularVelocity0);
    if ((flagsW & ANGULAR_W) != ANGULAR_W) {
        relativeVelocityW += dotW(linear, velocity1) - dotW(linear, velocity0);
    }

    const auto one = _mm_set1_ps(1.f);
    const auto biasFactor = _mm_set1_ps(0.2 / timeStep);

    const auto effMass = one / invEffMass;
    auto lambda = (relativeVelocityW - targetVelocity + biasFactor * c) * effMass;

    if (flagsW & SOFT_W) {
        const auto timeStepW = _mm_set1_ps(timeStep);
        const auto two = _mm_set1_ps(2.f);
        const auto twoPi = _mm_set1_ps(2.f * glm::pi<float>());

        auto angularFreq = twoPi * frequency;
        auto stiffness = angularFreq * angularFreq * effMass;
        auto damping = two * angularFreq * dampingRatio * effMass;
        auto gamma = one / (damping + timeStepW * stiffness);
        auto beta = timeStepW * stiffness * gamma;
        auto lambdaSoft = (relativeVelocityW + beta * c / timeStepW) / (invEffMass + gamma / timeStepW);

        auto soft = flagsW & SOFT_W;
        unsigned char* softWBytes = reinterpret_cast<unsigned char*>(&soft);
        auto softMask = _mm_cmpeq_epi32(_mm_setzero_si128(), _mm_set_epi32(softWBytes[3], softWBytes[2], softWBytes[1], softWBytes[0]));
        lambda = _mm_blendv_ps(lambdaSoft, lambda, _mm_castsi128_ps(softMask));
    }

    auto prevLambda = totalLambda;

    if (flagsW & LIMITED_W) {
        totalLambda += lambda;
        totalLambda = _mm_min_ps(_mm_max_ps(totalLambda, min), max);
        lambda = totalLambda - prevLambda;
    }
    else {
        totalLambda += lambda;
    }

    totalLambda = _mm_blendv_ps(prevLambda, totalLambda, invEffMassMask);

    if ((flagsW & ANGULAR_W) != ANGULAR_W) {
        velocity0 += lambda * invMass0 * linear;
        velocity1 -= lambda * invMass1 * linear;
    }

    angularVelocity0 += lambda * angular0t;
    angularVelocity1 -= lambda * angular1t;

    for (int i = 0; i < 4; ++i) {
        if (!invEffMass.m128_f32[i]) continue;

        if (dynamic0[i] && !dynamic0[i]->isKinematic) {
            if (!(flags[i] & Constraint1D::ANGULAR))
                velocity0.get(dynamic0[i]->velocity, i);
            angularVelocity0.get(dynamic0[i]->angularVelocity, i);
        }

        if (dynamic1[i] && !dynamic1[i]->isKinematic) {
            if (!(flags[i] & Constraint1D::ANGULAR))
                velocity1.get(dynamic1[i]->velocity, i);
            angularVelocity1.get(dynamic1[i]->angularVelocity, i);
        }
    }
}
