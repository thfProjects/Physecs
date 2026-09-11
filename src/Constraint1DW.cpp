#include "Constraint1DW.h"
#include "Constraint1DFlags.h"

namespace physecs {

template<auto member, typename T>
__forceinline Vec3W gatherVec3W(const T* data, const BodyId (&bodies)[4]) {
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
__forceinline void scatterVec3W(Vec3W v, T* data, const BodyId (&bodies)[4]) {
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
__forceinline void physecs::Constraint1DW<flags>::preSolve(VelocityData* velocities, PseudoVelocityData* pseudoVelocities, float timeStep) {
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

    FloatW invEffMass = dotW(angular0, angular0) + dotW(angular1, angular1);
    if constexpr (!(flags & ANGULAR)) {
        invEffMass += dotW(linear0, linear0) + dotW(linear1, linear1);
    }

    const FloatW timeStepW = _mm_set1_ps(timeStep);
    const FloatW one = _mm_set1_ps(1.f);

    if constexpr (flags & SOFT) {
        const FloatW stiffness = timeStepW * springParams.stiffness;
        const FloatW damping = timeStepW * springParams.damping;
        springParams.cfm = one / (damping + timeStepW * stiffness + _mm_set1_ps(1e-8f));
        springParams.erp = stiffness * springParams.cfm;

        effMass = one / (invEffMass + springParams.cfm);

        return;
    }
    else {
        effMass = one / (invEffMass + _mm_set1_ps(1e-8f));
    }

    if constexpr (flags & LIMITED) {
        min *= timeStepW;
        max *= timeStepW;
    }

    // warm start
    const auto warmStartCMask = _mm_cmplt_ps(_mm_abs_ps(c), _mm_set1_ps(1e-4));
    const auto totalLambdaMask = _mm_cmplt_ps(_mm_abs_ps(totalLambda), _mm_set1_ps(10000.f));

    const auto warmStartMask = _mm_and_ps(warmStartCMask, totalLambdaMask);

    totalLambda = _mm_blendv_ps(_mm_setzero_ps(), totalLambda * _mm_set1_ps(0.5f), warmStartMask);

    if (!isZero(warmStartMask)) {
        if constexpr (!(flags & ANGULAR)) {
            velocity0 += totalLambda * linear0;
            velocity1 -= totalLambda * linear1;
        }

        angularVelocity0 += totalLambda * angular0;
        angularVelocity1 -= totalLambda * angular1;
    }

    if constexpr (!(flags & ANGULAR)) {
        scatterVec3W<&VelocityData::velocity>(velocity0, velocities, bodies0);
        scatterVec3W<&VelocityData::velocity>(velocity1, velocities, bodies1);
    }
    scatterVec3W<&VelocityData::angularVelocity>(angularVelocity0, velocities, bodies0);
    scatterVec3W<&VelocityData::angularVelocity>(angularVelocity1, velocities, bodies1);

    // pseudo velocities
    const auto pseudoVelocityMask = _mm_cmpneq_ps(c, _mm_setzero_ps());

    if (!isZero(pseudoVelocityMask)) {
        auto lambda = c * effMass;
        if constexpr (flags & LIMITED) {
            FloatW minMask = _mm_cmplt_ps(min, _mm_setzero_ps());
            FloatW maxMask = _mm_cmpgt_ps(max, _mm_setzero_ps());
            const FloatW lo = _mm_blendv_ps(_mm_set1_ps(std::numeric_limits<float>::lowest()), _mm_setzero_ps(), minMask);
            const FloatW hi = _mm_blendv_ps(_mm_set1_ps(std::numeric_limits<float>::max()), _mm_setzero_ps(), maxMask);
            lambda = _mm_min_ps(_mm_max_ps(lambda, lo), hi);
        }

        lambda = _mm_blendv_ps(_mm_setzero_ps(), lambda, pseudoVelocityMask);

        if constexpr (!(flags & ANGULAR)) {
            pseudoVelocity0 += lambda * linear0;
            pseudoVelocity1 -= lambda * linear1;
        }

        pseudoAngularVelocity0 += lambda * angular0;
        pseudoAngularVelocity1 -= lambda * angular1;
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
__forceinline void physecs::Constraint1DW<flags>::solve(VelocityData* velocities, float baumgarteFactor) {
    Vec3W velocity0, velocity1;
    if constexpr (!(flags & ANGULAR)) {
        velocity0 = gatherVec3W<&VelocityData::velocity>(velocities, bodies0);
        velocity1 = gatherVec3W<&VelocityData::velocity>(velocities, bodies1);
    }
    Vec3W angularVelocity0 = gatherVec3W<&VelocityData::angularVelocity>(velocities, bodies0);
    Vec3W angularVelocity1 = gatherVec3W<&VelocityData::angularVelocity>(velocities, bodies1);

    auto relativeVelocityW = dotW(angular1, angularVelocity1) - dotW(angular0, angularVelocity0);
    if constexpr (!(flags & ANGULAR)) {
        relativeVelocityW += dotW(linear1, velocity1) - dotW(linear0, velocity0);
    }

    FloatW lambda = (relativeVelocityW - targetVelocity + (flags & SOFT ? springParams.erp : _mm_set1_ps(baumgarteFactor)) * c) * effMass;

    auto prevLambda = totalLambda;

    if constexpr (flags & LIMITED) {
        totalLambda += lambda;
        totalLambda = _mm_min_ps(_mm_max_ps(totalLambda, min ), max );
        lambda = totalLambda - prevLambda;
    }
    else {
        totalLambda += lambda;
    }

     if constexpr (!(flags & ANGULAR)) {
        velocity0 += lambda * linear0;
        velocity1 -= lambda * linear1;
    }

    angularVelocity0 += lambda * angular0;
    angularVelocity1 -= lambda * angular1;

    if constexpr (!(flags & ANGULAR)) {
        scatterVec3W<&VelocityData::velocity>(velocity0, velocities, bodies0);
        scatterVec3W<&VelocityData::velocity>(velocity1, velocities, bodies1);
    }
    scatterVec3W<&VelocityData::angularVelocity>(angularVelocity0, velocities, bodies0);
    scatterVec3W<&VelocityData::angularVelocity>(angularVelocity1, velocities, bodies1);
}
