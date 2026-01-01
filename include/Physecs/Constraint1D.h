#pragma once

#include <glm/glm.hpp>

struct TransformComponent;

namespace physecs {

    enum ConstraintFlags {
        NONE = 0,
        SOFT = 1,
        ANGULAR = 1 << 1,
        LIMITED = 1 << 2
    };

    struct RigidBodyDynamicComponent;

    template<int flags>
    struct Constraint1D {
        RigidBodyDynamicComponent* dynamic0;
        RigidBodyDynamicComponent* dynamic1;
        glm::vec3 linear = glm::vec3(0);
        glm::vec3 angular0 = glm::vec3(0);
        glm::vec3 angular1 = glm::vec3(0);
        float targetVelocity = 0;
        float c = 0;
        float min = std::numeric_limits<float>::lowest();
        float max = std::numeric_limits<float>::max();
        float frequency = 0;
        float dampingRatio = 0;
        glm::vec3 angular0t = glm::vec3(0);
        glm::vec3 angular1t = glm::vec3(0);
        float invEffMass = 0;
        float totalLambda = 0;

        Constraint1D(RigidBodyDynamicComponent* dynamic0, RigidBodyDynamicComponent* dynamic1) : dynamic0(dynamic0), dynamic1(dynamic1) {};
        void preSolve();
        void solve(float timeStep = 0);
    };
}
