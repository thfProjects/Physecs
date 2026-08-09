#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace physecs {

    struct VelocityData {
        glm::vec3 velocity;
        glm::vec3 angularVelocity;
    };

    struct PseudoVelocityData {
        glm::vec3 pseudoVelocity;
        glm::vec3 pseudoAngularVelocity;
        int constraintCount;
    };

    struct MassData {
        float invMass;
        glm::mat3 invInertiaTensor;
    };

    struct TransformData {
        glm::vec3 deltaTranslation;
        glm::quat deltaRotation;
        glm::vec3 comWorld;
        glm::quat worldRotation;
    };

    constexpr float baumgarteBias = 0.2f;
}
