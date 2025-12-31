#include "Constraint1DW.h"
#include <Constraint1D.h>
#include "Components.h"
#include "Transform.h"

template<int flags>
void physecs::Constraint1DW<flags>::preSolve() {
    Vec3W position0, position1, velocity0, velocity1, angularVelocity0, angularVelocity1;
    QuatW orientation0, orientation1;
    FloatW invMass0 = _mm_setzero_ps(), invMass1 = _mm_setzero_ps();
    for (int i = 0; i < 4; ++i) {
        if (!transform0[i]) break;

        FloatW invI0[3] = { _mm_setzero_ps(), _mm_setzero_ps(), _mm_setzero_ps() };
        if (dynamic0[i] && !dynamic0[i]->isKinematic) {
            auto& invI = dynamic0[i]->invInertiaTensorWorld;
            invI0[0] = _mm_setr_ps(invI[0][0], invI[0][1], invI[0][2], 0);
            invI0[1] = _mm_setr_ps(invI[1][0], invI[1][1], invI[1][2], 0);
            invI0[2] = _mm_setr_ps(invI[2][0], invI[2][1], invI[2][2], 0);

            if constexpr (!(flags & ANGULAR)) {
                invMass0.m128_f32[i] = dynamic0[i]->invMass;
                velocity0.set(dynamic0[i]->velocity, i);
            }

            angularVelocity0.set(dynamic0[i]->angularVelocity, i);
        }

        FloatW invI1[3] = { _mm_setzero_ps(), _mm_setzero_ps(), _mm_setzero_ps() };
        if (dynamic1[i] && !dynamic1[i]->isKinematic) {
            auto& invI = dynamic1[i]->invInertiaTensorWorld;
            invI1[0] = _mm_setr_ps(invI[0][0], invI[0][1], invI[0][2], 0);
            invI1[1] = _mm_setr_ps(invI[1][0], invI[1][1], invI[1][2], 0);
            invI1[2] = _mm_setr_ps(invI[2][0], invI[2][1], invI[2][2], 0);

            if constexpr (!(flags & ANGULAR)) {
                invMass1.m128_f32[i] = dynamic1[i]->invMass;
                velocity1.set(dynamic1[i]->velocity, i);
            }

            angularVelocity1.set(dynamic1[i]->angularVelocity, i);
        }

        const FloatW angular0x = _mm_set1_ps(angular0.x.m128_f32[i]);
        const FloatW angular0y = _mm_set1_ps(angular0.y.m128_f32[i]);
        const FloatW angular0z = _mm_set1_ps(angular0.z.m128_f32[i]);
        const FloatW angular0ti = angular0x * invI0[0] + angular0y * invI0[1] + angular0z * invI0[2];

        angular0t.x.m128_f32[i] = angular0ti.m128_f32[0];
        angular0t.y.m128_f32[i] = angular0ti.m128_f32[1];
        angular0t.z.m128_f32[i] = angular0ti.m128_f32[2];

        const FloatW angular1x = _mm_set1_ps(angular1.x.m128_f32[i]);
        const FloatW angular1y = _mm_set1_ps(angular1.y.m128_f32[i]);
        const FloatW angular1z = _mm_set1_ps(angular1.z.m128_f32[i]);
        const FloatW angular1ti = angular1x * invI1[0] + angular1y * invI1[1] + angular1z * invI1[2];

        angular1t.x.m128_f32[i] = angular1ti.m128_f32[0];
        angular1t.y.m128_f32[i] = angular1ti.m128_f32[1];
        angular1t.z.m128_f32[i] = angular1ti.m128_f32[2];

        position0.set(transform0[i]->position, i);
        position1.set(transform1[i]->position, i);
        orientation0.set(transform0[i]->orientation, i);
        orientation1.set(transform1[i]->orientation, i);
    }

    invEffMass = dotW(angular0, angular0t) + dotW(angular1, angular1t);
    if constexpr (!(flags & ANGULAR)) {
        invEffMass += dotW(linear, linear) * (invMass0 + invMass1);
        linear0t = invMass0 * linear;
        linear1t = invMass1 * linear;
    }

    if (flags & SOFT) return;

    const auto half = _mm_set1_ps(0.5f);

    auto cMask = _mm_cmplt_ps(_mm_abs_ps(c), _mm_set1_ps(1e-4));
    const auto totalLambdaMask = _mm_cmplt_ps(_mm_abs_ps(totalLambda), _mm_set1_ps(10000.f));

    const auto warmStartMask = _mm_and_ps(cMask, totalLambdaMask);

    // warm start
    if (!isZero(warmStartMask)) {
        auto lambda = totalLambda * half;
        lambda = _mm_blendv_ps(_mm_setzero_ps(), lambda, warmStartMask);

        if constexpr (!(flags & ANGULAR)) {
            velocity0 += lambda * linear0t;
            velocity1 -= lambda * linear1t;
        }

        angularVelocity0 += lambda * angular0t;
        angularVelocity1 -= lambda * angular1t;

        totalLambda = _mm_blendv_ps(totalLambda, lambda, warmStartMask);
    }

    // position correction (NGS)
    const auto invEffMassMask = _mm_cmpneq_ps(invEffMass, _mm_setzero_ps());
    cMask = _mm_cmpneq_ps(c, _mm_setzero_ps());

    auto ngsMask = _mm_and_ps(cMask, invEffMassMask);

    if (!isZero(ngsMask)) {
        const auto factor = _mm_set1_ps(0.1f);

        auto lambda = factor * c / invEffMass;
        if constexpr (flags & LIMITED) {
            lambda = _mm_min_ps(_mm_max_ps(lambda, min), max);
        }

        lambda = _mm_blendv_ps(_mm_setzero_ps(), lambda, ngsMask);

        if constexpr (!(flags & ANGULAR)) {
            position0 += lambda * linear0t;
            position1 -= lambda * linear1t;
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
                if constexpr (!(flags & ANGULAR))
                    velocity0.get(dynamic0[i]->velocity, i);
                angularVelocity0.get(dynamic0[i]->angularVelocity, i);
            }

            if (dynamic1[i] && !dynamic1[i]->isKinematic) {
                if constexpr (!(flags & ANGULAR))
                    velocity1.get(dynamic1[i]->velocity, i);
                angularVelocity1.get(dynamic1[i]->angularVelocity, i);
            }
        }
    }
}

template<int flags>
void physecs::Constraint1DW<flags>::solve(float timeStep) {
    auto invEffMassMask = _mm_cmpneq_ps(invEffMass, _mm_setzero_ps());
    if (isZero(invEffMassMask)) return;

    Vec3W velocity0, velocity1, angularVelocity0, angularVelocity1;
    for (int i = 0; i < 4; ++i) {
        if (dynamic0[i] && !dynamic0[i]->isKinematic) {
            if constexpr (!(flags & ANGULAR))
                velocity0.set(dynamic0[i]->velocity, i);
            angularVelocity0.set(dynamic0[i]->angularVelocity, i);
        }

        if (dynamic1[i] && !dynamic1[i]->isKinematic) {
            if constexpr (!(flags & ANGULAR))
                velocity1.set(dynamic1[i]->velocity, i);
            angularVelocity1.set(dynamic1[i]->angularVelocity, i);
        }
    }

    auto relativeVelocityW = dotW(angular1, angularVelocity1) - dotW(angular0, angularVelocity0);
    if constexpr (!(flags & ANGULAR)) {
        relativeVelocityW += dotW(linear, velocity1) - dotW(linear, velocity0);
    }

    const auto one = _mm_set1_ps(1.f);
    const auto biasFactor = _mm_set1_ps(0.2 / timeStep);

    const auto effMass = one / invEffMass;


    const auto timeStepW = _mm_set1_ps(timeStep);

    FloatW lambda;
    if constexpr (flags & SOFT) {
        const auto two = _mm_set1_ps(2.f);
        const auto twoPi = _mm_set1_ps(2.f * glm::pi<float>());

        auto angularFreq = twoPi * frequency;
        auto stiffness = angularFreq * angularFreq * effMass;
        auto damping = two * angularFreq * dampingRatio * effMass;
        auto gamma = one / (damping + timeStepW * stiffness);
        auto beta = timeStepW * stiffness * gamma;
        lambda = (relativeVelocityW + beta * c / timeStepW) / (invEffMass + gamma / timeStepW);
    }
    else {
        lambda = (relativeVelocityW - targetVelocity + biasFactor * c) * effMass;
    }

    auto prevLambda = totalLambda;

    if constexpr (flags & LIMITED) {
        totalLambda += lambda;
        totalLambda = _mm_min_ps(_mm_max_ps(totalLambda, min * timeStepW), max * timeStepW);
        lambda = totalLambda - prevLambda;
    }
    else {
        totalLambda += lambda;
    }

    totalLambda = _mm_blendv_ps(prevLambda, totalLambda, invEffMassMask);

     if constexpr (!(flags & ANGULAR)) {
        velocity0 += lambda * linear0t;
        velocity1 -= lambda * linear1t;
    }

    angularVelocity0 += lambda * angular0t;
    angularVelocity1 -= lambda * angular1t;

    for (int i = 0; i < 4; ++i) {
        if (!invEffMass.m128_f32[i]) continue;

        if (dynamic0[i] && !dynamic0[i]->isKinematic) {
            if constexpr (!(flags & ANGULAR))
                velocity0.get(dynamic0[i]->velocity, i);
            angularVelocity0.get(dynamic0[i]->angularVelocity, i);
        }

        if (dynamic1[i] && !dynamic1[i]->isKinematic) {
            if constexpr (!(flags & ANGULAR))
                velocity1.get(dynamic1[i]->velocity, i);
            angularVelocity1.get(dynamic1[i]->angularVelocity, i);
        }
    }
}
