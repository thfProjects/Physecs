#pragma once

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
}
