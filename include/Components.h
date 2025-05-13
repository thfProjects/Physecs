#pragma once

#include "Colliders.h"

namespace physecs {
    struct RigidBodyCollisionComponent {
        std::vector<Collider> colliders;
    };

    struct RigidBodyDynamicComponent {
        float invMass;
        glm::vec3 com;
        glm::vec3 velocity;
        glm::mat3 invInertiaTensor;
        glm::vec3 angularVelocity;
        bool isKinematic;
        glm::mat3 invInertiaTensorWorld;
    };
}
