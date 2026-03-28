#include "Constraint1DW.h"
#include <Constraint1D.h>
#include "SolverData.h"

template<int flags>
void physecs::Constraint1DW<flags>::preSolve(const MassData* masses) {
    FloatW invMass0 = _mm_setzero_ps(), invMass1 = _mm_setzero_ps();
    for (int i = 0; i < 4; ++i) {
        const int b0 = bodies0[i];
        const int b1 = bodies1[i];

        FloatW invI0[3] = { _mm_setzero_ps(), _mm_setzero_ps(), _mm_setzero_ps() };
        if (b0 >= 0) {
            auto& invI = masses[b0].invInertiaTensor;
            invI0[0] = _mm_setr_ps(invI[0][0], invI[0][1], invI[0][2], 0);
            invI0[1] = _mm_setr_ps(invI[1][0], invI[1][1], invI[1][2], 0);
            invI0[2] = _mm_setr_ps(invI[2][0], invI[2][1], invI[2][2], 0);

            if constexpr (!(flags & ANGULAR)) {
                invMass0.m128_f32[i] = masses[b0].invMass;
            }
        }

        FloatW invI1[3] = { _mm_setzero_ps(), _mm_setzero_ps(), _mm_setzero_ps() };
        if (b1 >= 0) {
            auto& invI = masses[b1].invInertiaTensor;
            invI1[0] = _mm_setr_ps(invI[0][0], invI[0][1], invI[0][2], 0);
            invI1[1] = _mm_setr_ps(invI[1][0], invI[1][1], invI[1][2], 0);
            invI1[2] = _mm_setr_ps(invI[2][0], invI[2][1], invI[2][2], 0);

            if constexpr (!(flags & ANGULAR)) {
                invMass1.m128_f32[i] = masses[b1].invMass;
            }
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
    }

    invEffMass = dotW(angular0, angular0t) + dotW(angular1, angular1t);
    if constexpr (!(flags & ANGULAR)) {
        invEffMass += dotW(linear, linear) * (invMass0 + invMass1);
        linear0t = invMass0 * linear;
        linear1t = invMass1 * linear;
    }
}

template<int flags>
void physecs::Constraint1DW<flags>::correctPositionError(PseudoVelocityData *pseudoVelocities) const {
    if constexpr (flags & SOFT) return;

    Vec3W pseudoVelocity0, pseudoVelocity1, pseudoAngularVelocity0, pseudoAngularVelocity1;
    for (int i = 0; i < 4; ++i) {
        const int b0 = bodies0[i];
        const int b1 = bodies1[i];

        if (b0 >= 0) {
            if constexpr (!(flags & ANGULAR))
                pseudoVelocity0.set(pseudoVelocities[b0].pseudoVelocity, i);
            pseudoAngularVelocity0.set(pseudoVelocities[b0].pseudoAngularVelocity, i);
        }

        if (b1 >= 0) {
            if constexpr (!(flags & ANGULAR))
                pseudoVelocity1.set(pseudoVelocities[b1].pseudoVelocity, i);
            pseudoAngularVelocity1.set(pseudoVelocities[b1].pseudoAngularVelocity, i);
        }
    }

    const auto invEffMassMask = _mm_cmpneq_ps(invEffMass, _mm_setzero_ps());
    const auto cMask = _mm_cmpneq_ps(c, _mm_setzero_ps());

    auto ngsMask = _mm_and_ps(cMask, invEffMassMask);

    if (!isZero(ngsMask)) {
        auto lambda = c / invEffMass;
        if constexpr (flags & LIMITED) {
            lambda = _mm_min_ps(_mm_max_ps(lambda, min), max);
        }

        lambda = _mm_blendv_ps(_mm_setzero_ps(), lambda, ngsMask);

        if constexpr (!(flags & ANGULAR)) {
            pseudoVelocity0 += lambda * linear0t;
            pseudoVelocity1 -= lambda * linear1t;
        }

        pseudoAngularVelocity0 += lambda * angular0t;
        pseudoAngularVelocity1 -= lambda * angular1t;
    }

    for (int i = 0; i < 4; ++i) {
        const int b0 = bodies0[i];
        const int b1 = bodies1[i];

        if (b0 >= 0) {
            if constexpr (!(flags & ANGULAR)) {
                pseudoVelocity0.get(pseudoVelocities[b0].pseudoVelocity, i);
            }
            pseudoAngularVelocity0.get(pseudoVelocities[b0].pseudoAngularVelocity, i);

            if (ngsMask.m128_i32[i]) ++pseudoVelocities[b0].constraintCount;
        }

        if (b1 >= 0) {
            if constexpr (!(flags & ANGULAR)) {
                pseudoVelocity1.get(pseudoVelocities[b1].pseudoVelocity, i);
            }
            pseudoAngularVelocity1.get(pseudoVelocities[b1].pseudoAngularVelocity, i);

            if (ngsMask.m128_i32[i]) ++pseudoVelocities[b1].constraintCount;
        }
    }
}

template<int flags>
void physecs::Constraint1DW<flags>::solve(VelocityData* velocities, float timeStep, bool warmStart) {
    Vec3W velocity0, velocity1, angularVelocity0, angularVelocity1;
    for (int i = 0; i < 4; ++i) {
        const int b0 = bodies0[i];
        const int b1 = bodies1[i];

        if (b0 >= 0) {
            if constexpr (!(flags & ANGULAR))
                velocity0.set(velocities[b0].velocity, i);
            angularVelocity0.set(velocities[b0].angularVelocity, i);
        }

        if (b1 >= 0) {
            if constexpr (!(flags & ANGULAR))
                velocity1.set(velocities[b1].velocity, i);
            angularVelocity1.set(velocities[b1].angularVelocity, i);
        }
    }

    if constexpr (!(flags & SOFT)) {
        if (warmStart) {
            const auto half = _mm_set1_ps(0.5f);

            auto cMask = _mm_cmplt_ps(_mm_abs_ps(c), _mm_set1_ps(1e-4));
            const auto totalLambdaMask = _mm_cmplt_ps(_mm_abs_ps(totalLambda), _mm_set1_ps(10000.f));

            const auto warmStartMask = _mm_and_ps(cMask, totalLambdaMask);

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
        }
    }

    auto invEffMassMask = _mm_cmpneq_ps(invEffMass, _mm_setzero_ps());
    if (isZero(invEffMassMask)) return;

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

        const int b0 = bodies0[i];
        const int b1 = bodies1[i];

        if (b0 >= 0) {
            if constexpr (!(flags & ANGULAR))
                velocity0.get(velocities[b0].velocity, i);
            angularVelocity0.get(velocities[b0].angularVelocity, i);
        }

        if (b1 >= 0) {
            if constexpr (!(flags & ANGULAR))
                velocity1.get(velocities[b1].velocity, i);
            angularVelocity1.get(velocities[b1].angularVelocity, i);
        }
    }
}
