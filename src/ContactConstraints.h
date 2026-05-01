#pragma once

#include "Components.h"
#include "Transform.h"

namespace physecs {

    struct MassData;
    struct VelocityData;

    struct ContactPointConstraint {
        glm::vec3 r0;
        glm::vec3 r1;
        glm::vec3 r0xn;
        glm::vec3 r1xn;
        float targetVelocity;
        float c;
        float totalLambda;
        glm::vec3 r0xnt;
        glm::vec3 r1xnt;
        float invEffMass;
        float distToFrictionAnchor;
    };

    struct FrictionConstraints {
        glm::vec3 r0;
        glm::vec3 r1;
        glm::vec3 t;
        glm::vec3 r0xt;
        glm::vec3 r1xt;
        float totalLambda;
        float totalLambdaTwist;
        glm::vec3 r0xtt;
        glm::vec3 r1xtt;
        glm::vec3 n0t;
        glm::vec3 n1t;
        float invEffMass;
        float invEffMassTwist;
    };

    struct ContactConstraints {
        TransformComponent& transform0;
        TransformComponent& transform1;
        RigidBodyDynamicComponent* dynamic0;
        RigidBodyDynamicComponent* dynamic1;
        int b0;
        int b1;
        glm::vec3 n;
        float friction;
        bool isSoft;
        float stiffness;
        float damping;
        int numPoints;
        ContactPointConstraint contactPointConstraints[4];
        FrictionConstraints frictionConstraints;

        void preSolve(const MassData* masses);
        void solve(VelocityData* velocities, bool useBias, float timeStep = 0);
    };
}

