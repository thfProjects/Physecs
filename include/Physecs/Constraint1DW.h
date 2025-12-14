#pragma once

#include "SIMD.h"

struct TransformComponent;

namespace physecs {

    struct RigidBodyDynamicComponent;

    struct Constraint1DW {
        enum FlagsW : unsigned int {
            SOFT_W = 1 | 1 << 8 | 1 << 16 | 1 << 24,
            ANGULAR_W = 1 << 1 | 1 << 9 | 1 << 17 | 1 << 25,
            LIMITED_W = 1 << 2 | 1 << 10 | 1 << 18 | 1 << 26,
        };

        TransformComponent* transform0[4] = {};
        TransformComponent* transform1[4] = {};
        RigidBodyDynamicComponent* dynamic0[4] = {};
        RigidBodyDynamicComponent* dynamic1[4] = {};
        Vec3W linear;
        Vec3W angular0;
        Vec3W angular1;
        FloatW targetVelocity = _mm_setzero_ps();
        FloatW c = _mm_setzero_ps();
        FloatW min = _mm_set1_ps(std::numeric_limits<float>::lowest());
        FloatW max = _mm_set1_ps(std::numeric_limits<float>::max());
        unsigned char flags[4] = {};
        FloatW frequency = _mm_setzero_ps();
        FloatW dampingRatio = _mm_setzero_ps();
        Vec3W linear0t;
        Vec3W linear1t;
        Vec3W angular0t;
        Vec3W angular1t;
        FloatW invEffMass = _mm_setzero_ps();
        FloatW totalLambda = _mm_setzero_ps();

        void preSolve();
        void solve(float timeStep);
    };
}

