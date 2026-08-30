#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "SIMD.h"

namespace physecs {

    struct VelocityData {
        FloatW velocity = _mm_setzero_ps();
        FloatW angularVelocity = _mm_setzero_ps();
    };

    struct PseudoVelocityData {
        FloatW pseudoVelocity = _mm_setzero_ps();
        FloatW pseudoAngularVelocity = _mm_setzero_ps();
        int constraintCount = 0;
    };

    struct MassData {
        FloatW invInertiaTensorAndMass = _mm_setzero_ps();
    };

    struct TransformData {
        glm::vec3 comWorld = glm::vec3(0);
        glm::mat3 worldRotation = glm::mat3(1);
    };

    using BodyId = int;
    constexpr BodyId INVALID_BODY_ID = -1;

    constexpr float baumgarteBias = 0.2f;
}
