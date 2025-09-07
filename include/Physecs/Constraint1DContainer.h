#pragma once

#include "Components.h"

struct TransformComponent;

namespace physecs {
    class Constraint1DContainer {

        friend class Constraint1DViewer;

        struct TransformComponents {
            TransformComponent& transform0;
            TransformComponent& transform1;

            TransformComponents (TransformComponent& t0, TransformComponent& t1) : transform0(t0), transform1(t1) {}
        };

        struct DynamicComponents {
            RigidBodyDynamicComponent* dynamic0;
            RigidBodyDynamicComponent* dynamic1;

            DynamicComponents (RigidBodyDynamicComponent* d0, RigidBodyDynamicComponent* d1) : dynamic0(d0), dynamic1(d1) {}
        };

        std::vector<TransformComponents> transformComponentsBuffer;
        std::vector<DynamicComponents> dynamicComponentsBuffer;
        std::vector<glm::vec3> linearBuffer;
        std::vector<glm::vec3> angular0Buffer;
        std::vector<glm::vec3> angular1Buffer;
        std::vector<float> targetVelocityBuffer;
        std::vector<float> cBuffer;
        std::vector<float> minBuffer;
        std::vector<float> maxBuffer;
        std::vector<char> flagsBuffer;
        std::vector<float> frequencyBuffer;
        std::vector<float> dampingRatioBuffer;
        std::vector<glm::vec3> angular0tBuffer;
        std::vector<glm::vec3> angular1tBuffer;
        std::vector<float> invEffMassBuffer;
        std::vector<float> totalLambdaBuffer;

    public:
        void pushBack(TransformComponent& transform0, TransformComponent& transform1, RigidBodyDynamicComponent* dynamic0, RigidBodyDynamicComponent* dynamic1);
        void clear();
        void preSolve();
        void solve(bool useBias, float timeStep);
    };

    struct Constraint1DView {
        glm::vec3& n;
        glm::vec3& r0xn;
        glm::vec3& r1xn;
        float& targetVelocity;
        float& c;
        float& min;
        float& max;
        char& flags;
        float& frequency;
        float& dampingRatio;
    };

    class Constraint1DViewer {
        int baseIndex;
        Constraint1DContainer& container;

    public:
        Constraint1DViewer(int baseIndex, Constraint1DContainer& container) : baseIndex(baseIndex), container(container) {}
        Constraint1DView operator[] (int index) const {
            const int i = baseIndex + index;
            return {
                container.linearBuffer[i],
                container.angular0Buffer[i],
                container.angular1Buffer[i],
                container.targetVelocityBuffer[i],
                container.cBuffer[i],
                container.minBuffer[i],
                container.maxBuffer[i],
                container.flagsBuffer[i],
                container.frequencyBuffer[i],
                container.dampingRatioBuffer[i]
            };
        }
    };
}

