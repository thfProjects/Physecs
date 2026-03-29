#pragma once

#include <glm/glm.hpp>

namespace physecs {

    struct VelocityData;
    struct PseudoVelocityData;
    struct MassData;

    enum ConstraintFlags {
        NONE = 0,
        SOFT = 1,
        ANGULAR = 1 << 1,
        LIMITED = 1 << 2
    };

    template<int flags>
    struct Constraint1D {
        int b0 = -1;
        int b1 = -1;
        glm::vec3 linear = glm::vec3(0);
        glm::vec3 angular0 = glm::vec3(0);
        glm::vec3 angular1 = glm::vec3(0);
        float targetVelocity = 0;
        float c = 0;
        float min = std::numeric_limits<float>::lowest();
        float max = std::numeric_limits<float>::max();
        float frequency = 0;
        float dampingRatio = 0;
        glm::vec3 linear0t = glm::vec3(0);
        glm::vec3 linear1t = glm::vec3(0);
        glm::vec3 angular0t = glm::vec3(0);
        glm::vec3 angular1t = glm::vec3(0);
        float invEffMass = 0;
        float totalLambda = 0;

        Constraint1D(int b0, int b1) : b0(b0), b1(b1) {};
        void preSolve(const MassData* masses, PseudoVelocityData* pseudoVelocities);
        void solve(VelocityData* velocities, float timeStep, bool warmStart);
    };
}
