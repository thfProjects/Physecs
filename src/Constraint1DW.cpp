#include "Constraint1DW.h"
#include "Constraint1DFlags.h"
#include "PhysecsPlatform.h"

namespace physecs {

template<auto member, typename T>
PHYSECS_FORCE_INLINE Vec3W gatherVec3W(T* const (&ptrs)[4]) {
    const FloatW& row0 = ptrs[0]->*member;
    const FloatW& row1 = ptrs[1]->*member;
    const FloatW& row2 = ptrs[2]->*member;
    const FloatW& row3 = ptrs[3]->*member;

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

// writes garbage to 4th lane of each vector, fine as long as nothing reads it
template<auto member, typename T>
PHYSECS_FORCE_INLINE void scatterVec3W(Vec3W v, T* const (&ptrs)[4]) {
    const FloatW xy01 = _mm_unpacklo_ps(v.x, v.y); // x0 y0 x1 y1
    const FloatW xy23 = _mm_unpackhi_ps(v.x, v.y); // x2 y2 x3 y3

    ptrs[0]->*member = _mm_shuffle_ps(xy01, v.z, _MM_SHUFFLE(3, 0, 1, 0)); // x0 y0 z0 -
    ptrs[1]->*member = _mm_shuffle_ps(xy01, v.z, _MM_SHUFFLE(3, 1, 3, 2)); // x1 y1 z1 -
    ptrs[2]->*member = _mm_shuffle_ps(xy23, v.z, _MM_SHUFFLE(3, 2, 1, 0)); // x2 y2 z2 -
    ptrs[3]->*member = _mm_shuffle_ps(xy23, v.z, _MM_SHUFFLE(3, 3, 3, 2)); // x3 y3 z3 -
}

}

template<int flags>
PHYSECS_FORCE_INLINE void physecs::Constraint1DW<flags>::preSolve(VelocityData* __restrict velocities, PseudoVelocityData* __restrict pseudoVelocities, float timeStep) {
    VelocityData* const velocityPtrs0[4] = { velocities + bodies0[0], velocities + bodies0[1], velocities + bodies0[2], velocities + bodies0[3] };
    VelocityData* const velocityPtrs1[4] = { velocities + bodies1[0], velocities + bodies1[1], velocities + bodies1[2], velocities + bodies1[3] };
    PseudoVelocityData* const pseudoVelocityPtrs0[4] = { pseudoVelocities + bodies0[0], pseudoVelocities + bodies0[1], pseudoVelocities + bodies0[2], pseudoVelocities + bodies0[3] };
    PseudoVelocityData* const pseudoVelocityPtrs1[4] = { pseudoVelocities + bodies1[0], pseudoVelocities + bodies1[1], pseudoVelocities + bodies1[2], pseudoVelocities + bodies1[3] };

    Vec3W pseudoVelocity0, pseudoVelocity1, pseudoAngularVelocity0, pseudoAngularVelocity1;
    Vec3W velocity0, velocity1, angularVelocity0, angularVelocity1;
    if constexpr (!(flags & SOFT)) {
        if constexpr (!(flags & ANGULAR)) {
            velocity0 = gatherVec3W<&VelocityData::velocity>(velocityPtrs0);
            velocity1 = gatherVec3W<&VelocityData::velocity>(velocityPtrs1);
        }
        angularVelocity0 = gatherVec3W<&VelocityData::angularVelocity>(velocityPtrs0);
        angularVelocity1 = gatherVec3W<&VelocityData::angularVelocity>(velocityPtrs1);

        if constexpr (!(flags & ANGULAR)) {
            pseudoVelocity0 = gatherVec3W<&PseudoVelocityData::pseudoVelocity>(pseudoVelocityPtrs0);
            pseudoVelocity1 = gatherVec3W<&PseudoVelocityData::pseudoVelocity>(pseudoVelocityPtrs1);
        }
        pseudoAngularVelocity0 = gatherVec3W<&PseudoVelocityData::pseudoAngularVelocity>(pseudoVelocityPtrs0);
        pseudoAngularVelocity1 = gatherVec3W<&PseudoVelocityData::pseudoAngularVelocity>(pseudoVelocityPtrs1);
    }

    const FloatW timeStepW = _mm_set1_ps(timeStep);
    const FloatW one = _mm_set1_ps(1.f);

    if constexpr (flags & SOFT) {
        FloatW invEffMass = dotW(angular0, angular0) + dotW(angular1, angular1);
        if constexpr (!(flags & ANGULAR)) {
            invEffMass += dotW(linear0, linear0) + dotW(linear1, linear1);
        }

        const FloatW stiffness = timeStepW * springParams.stiffness;
        const FloatW damping = timeStepW * springParams.damping;
        springParams.cfm = one / (damping + timeStepW * stiffness + _mm_set1_ps(1e-8f));
        springParams.erp = stiffness * springParams.cfm;

        effMass = one / (invEffMass + springParams.cfm);

        return;
    }

    if constexpr (flags & LIMITED) {
        FloatW invEffMass = dotW(angular0, angular0) + dotW(angular1, angular1);
        if constexpr (!(flags & ANGULAR)) {
            invEffMass += dotW(linear0, linear0) + dotW(linear1, linear1);
        }
        effMass = one / (invEffMass + _mm_set1_ps(1e-8f));
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
        scatterVec3W<&VelocityData::velocity>(velocity0, velocityPtrs0);
        scatterVec3W<&VelocityData::velocity>(velocity1, velocityPtrs1);
    }
    scatterVec3W<&VelocityData::angularVelocity>(angularVelocity0, velocityPtrs0);
    scatterVec3W<&VelocityData::angularVelocity>(angularVelocity1, velocityPtrs1);

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
        scatterVec3W<&PseudoVelocityData::pseudoVelocity>(pseudoVelocity0, pseudoVelocityPtrs0);
        scatterVec3W<&PseudoVelocityData::pseudoVelocity>(pseudoVelocity1, pseudoVelocityPtrs1);
    }
    scatterVec3W<&PseudoVelocityData::pseudoAngularVelocity>(pseudoAngularVelocity0, pseudoVelocityPtrs0);
    scatterVec3W<&PseudoVelocityData::pseudoAngularVelocity>(pseudoAngularVelocity1, pseudoVelocityPtrs1);

    for (int i = 0; i < 4; ++i) {
        if (laneI32(pseudoVelocityMask, i)) {
            ++pseudoVelocityPtrs0[i]->constraintCount;
            ++pseudoVelocityPtrs1[i]->constraintCount;
        }
    }
}

template<int flags>
PHYSECS_FORCE_INLINE void physecs::Constraint1DW<flags>::solve(VelocityData* __restrict velocities, float baumgarteFactor) {
    VelocityData* const velocityPtrs0[4] = { velocities + bodies0[0], velocities + bodies0[1], velocities + bodies0[2], velocities + bodies0[3] };
    VelocityData* const velocityPtrs1[4] = { velocities + bodies1[0], velocities + bodies1[1], velocities + bodies1[2], velocities + bodies1[3] };

    Vec3W velocity0, velocity1;
    if constexpr (!(flags & ANGULAR)) {
        velocity0 = gatherVec3W<&VelocityData::velocity>(velocityPtrs0);
        velocity1 = gatherVec3W<&VelocityData::velocity>(velocityPtrs1);
    }
    Vec3W angularVelocity0 = gatherVec3W<&VelocityData::angularVelocity>(velocityPtrs0);
    Vec3W angularVelocity1 = gatherVec3W<&VelocityData::angularVelocity>(velocityPtrs1);

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
        scatterVec3W<&VelocityData::velocity>(velocity0, velocityPtrs0);
        scatterVec3W<&VelocityData::velocity>(velocity1, velocityPtrs1);
    }
    scatterVec3W<&VelocityData::angularVelocity>(angularVelocity0, velocityPtrs0);
    scatterVec3W<&VelocityData::angularVelocity>(angularVelocity1, velocityPtrs1);
}
