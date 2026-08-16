#pragma once

#include "SIMD.h"
#include "SolverData.h"

namespace physecs {
    template<int flags>
    struct Constraint1DW {
        BodyId bodies0[4] = { INVALID_BODY_ID, INVALID_BODY_ID, INVALID_BODY_ID, INVALID_BODY_ID };
        BodyId bodies1[4] = { INVALID_BODY_ID, INVALID_BODY_ID, INVALID_BODY_ID, INVALID_BODY_ID };
        Vec3W linear;
        Vec3W angular0;
        Vec3W angular1;
        FloatW targetVelocity = _mm_setzero_ps();
        FloatW c = _mm_setzero_ps();
        FloatW min = _mm_set1_ps(std::numeric_limits<float>::lowest());
        FloatW max = _mm_set1_ps(std::numeric_limits<float>::max());
        FloatW stiffness = _mm_setzero_ps();
        FloatW damping = _mm_setzero_ps();
        Vec3W linear0t;
        Vec3W linear1t;
        Vec3W angular0t;
        Vec3W angular1t;
        FloatW invEffMass = _mm_setzero_ps();
        FloatW totalLambda = _mm_setzero_ps();

        void preSolve(const MassData* masses, VelocityData* velocities, PseudoVelocityData* pseudoVelocities);
        void solve(VelocityData* velocities, float timeStep, bool useBias);
    };
}

