#pragma once

#include "SolverData.h"

namespace physecs {

    struct MassData;
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
        ContactManifoldData* contactManifoldData;
        BodyId b0;
        BodyId b1;
        float invMass0;
        float invMass1;
        glm::vec3 n;
        float friction;
        bool isSoft;
        float stiffness;
        float damping;
        int numPoints;
        ContactPointConstraint contactPointConstraints[4];
        FrictionConstraints frictionConstraints;

        void preSolve(const MassData* masses, VelocityData* velocities);
        void solve(VelocityData* velocities, bool useBias, float timeStep = 0);
    };
}

