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
        float invMass = 0;
        glm::mat3 invInertiaTensor = glm::mat3(0);
    };

    struct TransformData {
        glm::vec3 deltaTranslation = glm::vec3(0);
        glm::mat3 deltaRotation = glm::mat3(1);
        glm::vec3 comWorld = glm::vec3(0);
        glm::quat worldRotation = glm::quat(1, 0, 0, 0);
    };

    using BodyId = int;
    constexpr BodyId INVALID_BODY_ID = -1;

    constexpr float baumgarteBias = 0.2f;
}
