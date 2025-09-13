#pragma once

#include <entt.hpp>

#include "Components.h"

struct TransformComponent;

namespace physecs {

    class Constraint1DSoa {

        friend class Constraint1DViewer;

        struct TransformComponents {
            TransformComponent* transform0;
            TransformComponent* transform1;

            TransformComponents () : transform0 (nullptr), transform1 (nullptr) {}
            TransformComponents (TransformComponent* t0, TransformComponent* t1) : transform0(t0), transform1(t1) {}
        };

        struct DynamicComponents {
            RigidBodyDynamicComponent* dynamic0;
            RigidBodyDynamicComponent* dynamic1;

            DynamicComponents () : dynamic0(nullptr), dynamic1(nullptr) {}
            DynamicComponents (RigidBodyDynamicComponent* d0, RigidBodyDynamicComponent* d1) : dynamic0(d0), dynamic1(d1) {}
        };

        template<typename T>
        class Field {
            T* data;
        public:
            T& operator[](int i) { return data[i]; }
            Field () : data(nullptr) {}
            Field (Field&& other) noexcept : data(other.data) {
                other.data = nullptr;
            }
            void reallocate(int oldSize, int newSize) {
                T* newBuffer = static_cast<T*>(_aligned_malloc(newSize * sizeof(T), 16));
                std::copy(data, data + oldSize, newBuffer);
                _aligned_free(data);
                data = newBuffer;
            }
            ~Field() { _aligned_free(data); }
        };

        Field<TransformComponents> transformComponentsBuffer;
        Field<DynamicComponents> dynamicComponentsBuffer;
        Field<glm::vec3> linearBuffer;
        Field<glm::vec3> angular0Buffer;
        Field<glm::vec3> angular1Buffer;
        Field<float> targetVelocityBuffer;
        Field<float> cBuffer;
        Field<float> minBuffer;
        Field<float> maxBuffer;
        Field<char> flagsBuffer;
        Field<float> frequencyBuffer;
        Field<float> dampingRatioBuffer;
        Field<glm::vec3> angular0tBuffer;
        Field<glm::vec3> angular1tBuffer;
        Field<float> invEffMassBuffer;
        Field<float> totalLambdaBuffer;

        int size = 0;
        int capacity = 0;

    public:
        void preSolve();
        void solve(bool useBias, float timeStep);
        void solveSimd(float timeStep);
        void pushBack(TransformComponent& transform0, TransformComponent& transform1, RigidBodyDynamicComponent* dynamic0, RigidBodyDynamicComponent* dynamic1);
        void clear();
        int getSize() const { return size; };
    };

    class Constraint1DContainer {

        friend class Constraint1DViewer;

        struct Constraint1DMapper {
            int color;
            int index;
        };

        constexpr static int maxColors = 8;

        std::vector<Constraint1DSoa> constraintColors;
        std::unordered_map<entt::entity, std::uint32_t> colorBitsets;
        std::vector<Constraint1DMapper> mappers;
        Constraint1DSoa sequential;

    public:
        void preSolve();
        void solve(bool useBias, float timeStep);
        void pushBack(entt::entity entity0, entt::entity entity1, TransformComponent& transform0, TransformComponent& transform1, RigidBodyDynamicComponent* dynamic0, RigidBodyDynamicComponent* dynamic1);
        void clear();
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
            const int mapperIndex = baseIndex + index;
            auto& [color, i] = container.mappers[mapperIndex];
            auto& soa = color == -1 ? container.sequential : container.constraintColors[color];
            return {
                soa.linearBuffer[i],
                soa.angular0Buffer[i],
                soa.angular1Buffer[i],
                soa.targetVelocityBuffer[i],
                soa.cBuffer[i],
                soa.minBuffer[i],
                soa.maxBuffer[i],
                soa.flagsBuffer[i],
                soa.frequencyBuffer[i],
                soa.dampingRatioBuffer[i]
            };
        }
    };
}

