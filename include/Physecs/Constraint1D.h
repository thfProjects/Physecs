#pragma once

#include <glm/glm.hpp>

struct TransformComponent;

namespace physecs {

    struct RigidBodyDynamicComponent;

    struct Constraint1D {
        enum Flags {
            SOFT = 1,
            ANGULAR = 1 << 1,
            LIMITED = 1 << 2
        };

        TransformComponent& transform0;
        TransformComponent& transform1;
        RigidBodyDynamicComponent* dynamic0;
        RigidBodyDynamicComponent* dynamic1;
        glm::vec3 n;
        glm::vec3 r0xn;
        glm::vec3 r1xn;
        float targetVelocity;
        float c;
        float min;
        float max;
        char flags;
        float frequency = 0;
        float dampingRatio = 0;
        glm::vec3 r0xnt;
        glm::vec3 r1xnt;
        float invEffMass;
        float totalLambda;

        Constraint1D(TransformComponent& transform0, TransformComponent& transform1, RigidBodyDynamicComponent* dynamic0, RigidBodyDynamicComponent* dynamic1, glm::vec3 n, glm::vec3 r0xn, glm::vec3 r1xn, float targetVelocity, float c, float min, float max, char flags = 0, float totalLambda = 0):
        transform0(transform0), transform1(transform1), dynamic0(dynamic0), dynamic1(dynamic1), n(n), r0xn(r0xn), r1xn(r1xn), targetVelocity(targetVelocity), c(c), min(min), max(max), flags(flags), totalLambda(totalLambda) {}
        void prepare();
        void solve(bool useBias, float timeStep = 0);
        void warmStart();
    };
}
