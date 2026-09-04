#pragma once

#include "SolverData.h"

namespace physecs {

    struct VelocityData;
    struct PseudoVelocityData;

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
        float stiffness = 0;
        float damping = 0;
        float invEffMass = 0;
        float totalLambda = 0;

        Constraint1D(int b0, int b1, float initLambda) : b0(b0), b1(b1), totalLambda(initLambda) {};
        void preSolve(VelocityData* velocities, PseudoVelocityData* pseudoVelocities);
        void solve(VelocityData* velocities, float timeStep, bool useBias);
    };
}
