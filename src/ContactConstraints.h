#pragma once

#include "Components.h"
#include "Transform.h"

namespace physecs {

    struct ContactPointConstraints {
        glm::vec3 r0;
        glm::vec3 r1;
        glm::vec3 r0xn;
        glm::vec3 r1xn;
        glm::vec3 t;
        glm::vec3 r0xt;
        glm::vec3 r1xt;
        float targetVelocity;
        float c;
        float totalLambdaN;
        float totalLambdaT;
    };

    struct ContactConstraints {
        TransformComponent& transform0;
        TransformComponent& transform1;
        RigidBodyDynamicComponent* dynamic0;
        RigidBodyDynamicComponent* dynamic1;
        glm::vec3 n;
        float friction;
        bool isSoft;
        float frequency;
        float dampingRatio;
        int numPoints;
        ContactPointConstraints contactPointConstraints[4];

        void preSolve();
        void solve(bool useBias, float timeStep = 0);
    };
}

