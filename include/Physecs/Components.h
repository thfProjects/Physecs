#pragma once

#include "Colliders.h"

namespace physecs {
    struct RigidBodyCollisionComponent {
        std::vector<Collider> colliders;
    };

    struct MassProps {
        float invMass;
        glm::vec3 com;
        glm::vec3 invInertiaDiag;
        glm::quat principalAxes;
    };

    struct RigidBodyDynamicComponent {
        bool isKinematic;
        glm::vec3 velocity;
        glm::vec3 angularVelocity;
        MassProps massProps;
    };
}
