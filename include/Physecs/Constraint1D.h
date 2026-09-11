#pragma once

#include "SolverData.h"

namespace physecs {
    struct SpringParams {
        union {
            float stiffness;
            float erp;
        };

        union {
            float damping;
            float cfm;
        };

        SpringParams() : stiffness(0.f), damping(0.f) {}
    };

    template<int flags>
    struct Constraint1D {
        BodyId b0 = INVALID_BODY_ID;
        BodyId b1 = INVALID_BODY_ID;
        glm::vec3 linear0 = glm::vec3(0);
        glm::vec3 linear1 = glm::vec3(0);
        glm::vec3 angular0 = glm::vec3(0);
        glm::vec3 angular1 = glm::vec3(0);
        float targetVelocity = 0;
        float c = 0;
        float min = std::numeric_limits<float>::lowest();
        float max = std::numeric_limits<float>::max();
        SpringParams springParams;
        float effMass = 0;
        float totalLambda = 0;

        Constraint1D(int b0, int b1, float initLambda) : b0(b0), b1(b1), totalLambda(initLambda) {};
        void preSolve(VelocityData* velocities, PseudoVelocityData* pseudoVelocities, float timeStep);
        void solve(VelocityData* velocities, float baumgarteFactor);
    };
}
