#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace physecs {

    struct VelocityData {
        glm::vec3 velocity = glm::vec3(0);
        glm::vec3 angularVelocity = glm::vec3(0);
    };

    struct PseudoVelocityData {
        glm::vec3 pseudoVelocity = glm::vec3(0);
        glm::vec3 pseudoAngularVelocity = glm::vec3(0);
        int constraintCount = 0;
    };

    struct MassData {
        float invMass = 0;
        glm::mat3 invInertiaTensor = glm::mat3(0);
    };

    struct TransformData {
        glm::vec3 deltaTranslation = glm::vec3(0);
        glm::quat deltaRotation = glm::quat(1, 0, 0, 0);
        glm::vec3 comWorld = glm::vec3(0);
        glm::quat worldRotation = glm::quat(1, 0, 0, 0);
    };

    using BodyId = int;
    constexpr BodyId INVALID_BODY_ID = -1;

    constexpr float baumgarteBias = 0.2f;
}
