#pragma once

#include "SolverData.h"

namespace physecs {

    struct VelocityData;
    struct ContactManifoldData;

    struct ContactPointConstraint {
        glm::vec3 r0;
        glm::vec3 r1;
        glm::vec3 r0xn;
        glm::vec3 r1xn;
        float targetVelocity;
        float c;
        float totalLambda;
        float invEffMass;
        float distToFrictionAnchor;
    };

    struct FrictionConstraints {
        glm::vec3 r0;
        glm::vec3 r1;
        glm::vec3 t0;
        glm::vec3 t1;
        glm::vec3 r0xt;
        glm::vec3 r1xt;
        float totalLambda;
        float totalLambdaTwist;
        glm::vec3 n0;
        glm::vec3 n1;
        float invEffMass;
        float invEffMassTwist;
    };

    struct ContactConstraints {
        ContactManifoldData* contactManifoldData;
        BodyId b0;
        BodyId b1;
        glm::vec3 n;
        glm::vec3 n0;
        glm::vec3 n1;
        float friction;
        bool isSoft;
        float stiffness;
        float damping;
        int numPoints;
        ContactPointConstraint contactPointConstraints[4];
        FrictionConstraints frictionConstraints;

        void preSolve(VelocityData* velocities);
        void solve(VelocityData* velocities, bool useBias, float timeStep = 0);
    };
}

