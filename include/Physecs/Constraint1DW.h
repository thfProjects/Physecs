#pragma once

#include "SIMD.h"
#include "SolverData.h"

namespace physecs {
    struct SpringParamsW {
        union {
            FloatW stiffness;
            FloatW erp;
        };

        union {
            FloatW damping;
            FloatW cfm;
        };

        SpringParamsW() : stiffness(_mm_setzero_ps()), damping(_mm_setzero_ps()) {}
    };

    template<int flags>
    struct Constraint1DW {
        BodyId bodies0[4] = { INVALID_BODY_ID, INVALID_BODY_ID, INVALID_BODY_ID, INVALID_BODY_ID };
        BodyId bodies1[4] = { INVALID_BODY_ID, INVALID_BODY_ID, INVALID_BODY_ID, INVALID_BODY_ID };
        Vec3W linear0;
        Vec3W linear1;
        Vec3W angular0;
        Vec3W angular1;
        FloatW targetVelocity = _mm_setzero_ps();
        FloatW c = _mm_setzero_ps();
        FloatW min = _mm_set1_ps(std::numeric_limits<float>::lowest());
        FloatW max = _mm_set1_ps(std::numeric_limits<float>::max());
        SpringParamsW springParams;
        FloatW effMass = _mm_setzero_ps();
        FloatW totalLambda = _mm_setzero_ps();

        void preSolve(VelocityData* velocities, PseudoVelocityData* pseudoVelocities, float timeStep);
        void solve(VelocityData* velocities, float baumgarteFactor);
    };
}

