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
        TransformComponent& transform0;
        TransformComponent& transform1;
        RigidBodyDynamicComponent* dynamic0;
        RigidBodyDynamicComponent* dynamic1;
        glm::vec3 n = glm::vec3(0);
        glm::vec3 r0xn = glm::vec3(0);
        glm::vec3 r1xn = glm::vec3(0);
        float targetVelocity = 0;
        float c = 0;
        float min = std::numeric_limits<float>::lowest();
        float max = std::numeric_limits<float>::max();
        float frequency = 0;
        float dampingRatio = 0;
        glm::vec3 r0xnt = glm::vec3(0);
        glm::vec3 r1xnt = glm::vec3(0);
        float invEffMass = 0;
        float totalLambda = 0;

        Constraint1D(TransformComponent& transform0, TransformComponent& transform1, RigidBodyDynamicComponent* dynamic0, RigidBodyDynamicComponent* dynamic1) : transform0(transform0), transform1(transform1), dynamic0(dynamic0), dynamic1(dynamic1) {};
        void preSolve();
        void solve(float timeStep = 0);
    };
}
