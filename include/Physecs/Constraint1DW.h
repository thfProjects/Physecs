#pragma once

#include "SIMD.h"

namespace physecs {

    struct VelocityData;
    struct PseudoVelocityData;
    struct MassData;

    template<int flags>
    struct Constraint1DW {
        int bodies0[4] = { -1, -1, -1, -1 };
        int bodies1[4] = { -1, -1, -1, -1 };
        Vec3W linear;
        Vec3W angular0;
        Vec3W angular1;
        FloatW targetVelocity = _mm_setzero_ps();
        FloatW c = _mm_setzero_ps();
        FloatW min = _mm_set1_ps(std::numeric_limits<float>::lowest());
        FloatW max = _mm_set1_ps(std::numeric_limits<float>::max());
        FloatW frequency = _mm_setzero_ps();
        FloatW dampingRatio = _mm_setzero_ps();
        Vec3W linear0t;
        Vec3W linear1t;
        Vec3W angular0t;
        Vec3W angular1t;
        FloatW invEffMass = _mm_setzero_ps();
        FloatW totalLambda = _mm_setzero_ps();

        void preSolve(const MassData* masses);
        void correctPositionError(PseudoVelocityData* pseudoVelocities) const;
        void solve(VelocityData* velocities, float timeStep, bool warmStart);
    };
}

