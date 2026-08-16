#include "Constraint1DW.h"
#include <Constraint1D.h>

namespace physecs {

template<auto member, typename T>
Vec3W gatherVec3W(const T* data, const BodyId (&bodies)[4]) {
    const FloatW& row0 = data[bodies[0]].*member;
    const FloatW& row1 = data[bodies[1]].*member;
    const FloatW& row2 = data[bodies[2]].*member;
    const FloatW& row3 = data[bodies[3]].*member;

    const FloatW tmp0 = _mm_unpacklo_ps(row0, row1); // x0 x1 y0 y1
    const FloatW tmp1 = _mm_unpacklo_ps(row2, row3); // x2 x3 y2 y3
    const FloatW tmp2 = _mm_unpackhi_ps(row0, row1); // z0 z1 0 0
    const FloatW tmp3 = _mm_unpackhi_ps(row2, row3); // z2 z3 0 0

    return Vec3W(
        _mm_movelh_ps(tmp0, tmp1),
        _mm_movehl_ps(tmp1, tmp0),
        _mm_movelh_ps(tmp2, tmp3)
    );
}

template<auto member, typename T>
void scatterVec3W(Vec3W v, T* data, const BodyId (&bodies)[4]) {
    const FloatW tmp0 = _mm_unpacklo_ps(v.x, v.y);
    const FloatW tmp1 = _mm_unpacklo_ps(v.z, _mm_setzero_ps());
    const FloatW tmp2 = _mm_unpackhi_ps(v.x, v.y);
    const FloatW tmp3 = _mm_unpackhi_ps(v.z, _mm_setzero_ps());

    data[bodies[0]].*member = _mm_movelh_ps(tmp0, tmp1);
    data[bodies[1]].*member = _mm_movehl_ps(tmp1, tmp0);
    data[bodies[2]].*member = _mm_movelh_ps(tmp2, tmp3);
    data[bodies[3]].*member = _mm_movehl_ps(tmp3, tmp2);
}

}

template<int flags>
void physecs::Constraint1DW<flags>::preSolve(const MassData* masses, VelocityData* velocities, PseudoVelocityData* pseudoVelocities) {
    Vec3W pseudoVelocity0, pseudoVelocity1, pseudoAngularVelocity0, pseudoAngularVelocity1;
    Vec3W velocity0, velocity1, angularVelocity0, angularVelocity1;
    if constexpr (!(flags & SOFT)) {
        if constexpr (!(flags & ANGULAR)) {
            velocity0 = gatherVec3W<&VelocityData::velocity>(velocities, bodies0);
            velocity1 = gatherVec3W<&VelocityData::velocity>(velocities, bodies1);
        }
        angularVelocity0 = gatherVec3W<&VelocityData::angularVelocity>(velocities, bodies0);
        angularVelocity1 = gatherVec3W<&VelocityData::angularVelocity>(velocities, bodies1);

        if constexpr (!(flags & ANGULAR)) {
            pseudoVelocity0 = gatherVec3W<&PseudoVelocityData::pseudoVelocity>(pseudoVelocities, bodies0);
            pseudoVelocity1 = gatherVec3W<&PseudoVelocityData::pseudoVelocity>(pseudoVelocities, bodies1);
        }
        pseudoAngularVelocity0 = gatherVec3W<&PseudoVelocityData::pseudoAngularVelocity>(pseudoVelocities, bodies0);
        pseudoAngularVelocity1 = gatherVec3W<&PseudoVelocityData::pseudoAngularVelocity>(pseudoVelocities, bodies1);
    }

    FloatW invMass0 = _mm_setzero_ps(), invMass1 = _mm_setzero_ps();
    for (int i = 0; i < 4; ++i) {
        const BodyId b0 = bodies0[i];
        const BodyId b1 = bodies1[i];

        if constexpr (!(flags & ANGULAR)) {
            invMass0.m128_f32[i] = masses[b0].invMass;
            invMass1.m128_f32[i] = masses[b1].invMass;
        }

        FloatW invI0[3];
        {
            auto& invI = masses[b0].invInertiaTensor;
            invI0[0] = _mm_setr_ps(invI[0][0], invI[0][1], invI[0][2], 0);
            invI0[1] = _mm_setr_ps(invI[1][0], invI[1][1], invI[1][2], 0);
            invI0[2] = _mm_setr_ps(invI[2][0], invI[2][1], invI[2][2], 0);
        }

        FloatW invI1[3];
        {
            auto& invI = masses[b1].invInertiaTensor;
            invI1[0] = _mm_setr_ps(invI[0][0], invI[0][1], invI[0][2], 0);
            invI1[1] = _mm_setr_ps(invI[1][0], invI[1][1], invI[1][2], 0);
            invI1[2] = _mm_setr_ps(invI[2][0], invI[2][1], invI[2][2], 0);
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

    if constexpr (flags & SOFT) return;

    // warm start
    const auto warmStartCMask = _mm_cmplt_ps(_mm_abs_ps(c), _mm_set1_ps(1e-4));
    const auto totalLambdaMask = _mm_cmplt_ps(_mm_abs_ps(totalLambda), _mm_set1_ps(10000.f));

    const auto warmStartMask = _mm_and_ps(warmStartCMask, totalLambdaMask);

    totalLambda = _mm_blendv_ps(_mm_setzero_ps(), totalLambda * _mm_set1_ps(0.5f), warmStartMask);

    if (!isZero(warmStartMask)) {
        if constexpr (!(flags & ANGULAR)) {
            velocity0 += totalLambda * linear0t;
            velocity1 -= totalLambda * linear1t;
        }

        angularVelocity0 += totalLambda * angular0t;
        angularVelocity1 -= totalLambda * angular1t;
    }

    if constexpr (!(flags & ANGULAR)) {
        scatterVec3W<&VelocityData::velocity>(velocity0, velocities, bodies0);
        scatterVec3W<&VelocityData::velocity>(velocity1, velocities, bodies1);
    }
    scatterVec3W<&VelocityData::angularVelocity>(angularVelocity0, velocities, bodies0);
    scatterVec3W<&VelocityData::angularVelocity>(angularVelocity1, velocities, bodies1);

    // pseudo velocities
    const auto invEffMassMask = _mm_cmpneq_ps(invEffMass, _mm_setzero_ps());
    const auto pseudoVelocityCMask = _mm_cmpneq_ps(c, _mm_setzero_ps());

    const auto pseudoVelocityMask = _mm_and_ps(pseudoVelocityCMask, invEffMassMask);

    if (!isZero(pseudoVelocityMask)) {
        auto lambda = c / invEffMass;
        if constexpr (flags & LIMITED) {
            lambda = _mm_min_ps(_mm_max_ps(lambda, min), max);
        }

        lambda = _mm_blendv_ps(_mm_setzero_ps(), lambda, pseudoVelocityMask);

        if constexpr (!(flags & ANGULAR)) {
            pseudoVelocity0 += lambda * linear0t;
            pseudoVelocity1 -= lambda * linear1t;
        }

        pseudoAngularVelocity0 += lambda * angular0t;
        pseudoAngularVelocity1 -= lambda * angular1t;
    }

    if constexpr (!(flags & ANGULAR)) {
        scatterVec3W<&PseudoVelocityData::pseudoVelocity>(pseudoVelocity0, pseudoVelocities, bodies0);
        scatterVec3W<&PseudoVelocityData::pseudoVelocity>(pseudoVelocity1, pseudoVelocities, bodies1);
    }
    scatterVec3W<&PseudoVelocityData::pseudoAngularVelocity>(pseudoAngularVelocity0, pseudoVelocities, bodies0);
    scatterVec3W<&PseudoVelocityData::pseudoAngularVelocity>(pseudoAngularVelocity1, pseudoVelocities, bodies1);

    for (int i = 0; i < 4; ++i) {
        const BodyId b0 = bodies0[i];
        const BodyId b1 = bodies1[i];

        if (pseudoVelocityMask.m128_i32[i]) {
            ++pseudoVelocities[b0].constraintCount;
            ++pseudoVelocities[b1].constraintCount;
        }
    }
}

template<int flags>
void physecs::Constraint1DW<flags>::solve(VelocityData* velocities, float timeStep, bool useBias) {
    Vec3W velocity0, velocity1;
    if constexpr (!(flags & ANGULAR)) {
        velocity0 = gatherVec3W<&VelocityData::velocity>(velocities, bodies0);
        velocity1 = gatherVec3W<&VelocityData::velocity>(velocities, bodies1);
    }
    Vec3W angularVelocity0 = gatherVec3W<&VelocityData::angularVelocity>(velocities, bodies0);
    Vec3W angularVelocity1 = gatherVec3W<&VelocityData::angularVelocity>(velocities, bodies1);

    auto invEffMassMask = _mm_cmpneq_ps(invEffMass, _mm_setzero_ps());
    if (isZero(invEffMassMask)) return;

    auto relativeVelocityW = dotW(angular1, angularVelocity1) - dotW(angular0, angularVelocity0);
    if constexpr (!(flags & ANGULAR)) {
        relativeVelocityW += dotW(linear, velocity1) - dotW(linear, velocity0);
    }

    const auto one = _mm_set1_ps(1.f);
    const auto biasFactor = _mm_set1_ps((useBias ? baumgarteBias : 0.f) / timeStep);

    const auto effMass = one / invEffMass;

    const auto timeStepW = _mm_set1_ps(timeStep);

    FloatW lambda;
    if constexpr (flags & SOFT) {
        auto gamma = one / (damping + timeStepW * stiffness);
        auto beta = timeStepW * stiffness * gamma;
        lambda = (relativeVelocityW + beta * c / timeStepW) / (invEffMass + gamma / timeStepW);
    }
    else {
        lambda = (relativeVelocityW - targetVelocity + biasFactor * c) * effMass;
    }

    lambda = _mm_blendv_ps(_mm_setzero_ps(), lambda, invEffMassMask);

    auto prevLambda = totalLambda;

    if constexpr (flags & LIMITED) {
        totalLambda += lambda;
        totalLambda = _mm_min_ps(_mm_max_ps(totalLambda, min * timeStepW), max * timeStepW);
        lambda = totalLambda - prevLambda;
    }
    else {
        totalLambda += lambda;
    }

     if constexpr (!(flags & ANGULAR)) {
        velocity0 += lambda * linear0t;
        velocity1 -= lambda * linear1t;
    }

    angularVelocity0 += lambda * angular0t;
    angularVelocity1 -= lambda * angular1t;

    if constexpr (!(flags & ANGULAR)) {
        scatterVec3W<&VelocityData::velocity>(velocity0, velocities, bodies0);
        scatterVec3W<&VelocityData::velocity>(velocity1, velocities, bodies1);
    }
    scatterVec3W<&VelocityData::angularVelocity>(angularVelocity0, velocities, bodies0);
    scatterVec3W<&VelocityData::angularVelocity>(angularVelocity1, velocities, bodies1);
}
