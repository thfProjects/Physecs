#pragma once

#include "Colliders.h"

namespace physecs {
    struct RigidBodyCollisionComponent {
        std::vector<Collider> colliders;
    };

    struct RigidBodyDynamicComponent {
        bool isKinematic;
        glm::vec3 velocity;
        glm::vec3 angularVelocity;
        glm::vec3 pseudoVelocity;
        glm::vec3 pseudoAngularVelocity;
        int invPseudoVelocityScale;
        float invMass;
        glm::vec3 com;
        glm::mat3 invInertiaTensor;
        glm::mat3 invInertiaTensorWorld;
    };
}
