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
                std::fill(newBuffer + oldSize, newBuffer + newSize, T());
                _aligned_free(data);
                data = newBuffer;
            }
            T* get() { return data; }
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
        Field<unsigned char> flagsBuffer;
        Field<float> frequencyBuffer;
        Field<float> dampingRatioBuffer;
        Field<glm::vec3> angular0tBuffer;
        Field<glm::vec3> angular1tBuffer;
        Field<float> invEffMassBuffer;
        Field<float> totalLambdaBuffer;

        //temp values
        Field<glm::vec3> velocity0Buffer;
        Field<glm::vec3> velocity1Buffer;
        Field<glm::vec3> angularVelocity0Buffer;
        Field<glm::vec3> angularVelocity1Buffer;
        Field<float> invMass0Buffer;
        Field<float> invMass1Buffer;
        Field<glm::vec3> position0Buffer;
        Field<glm::vec3> position1Buffer;
        Field<glm::quat> orientation0Buffer;
        Field<glm::quat> orientation1Buffer;

        int size = 0;
        int capacity = 0;

    public:
        void preSolve();
        void preSolveSimd();
        void solve(bool useBias, float timeStep);
        void solveSimd(float timeStep);
        void pushBack(TransformComponent& transform0, TransformComponent& transform1, RigidBodyDynamicComponent* dynamic0, RigidBodyDynamicComponent* dynamic1);
        void fillDefaults();
        void clear();
        int getSize() const { return size; }
    };

    class Constraint1DContainer {

        friend class Constraint1DViewer;

        struct Constraint1DMapper {
            Constraint1DSoa* color;
            int index;

            Constraint1DMapper(Constraint1DSoa* color, int index) : color(color), index(index) {}
        };

        constexpr static int maxColors = 32;

        Constraint1DSoa constraintColors[maxColors];
        entt::dense_map<entt::entity, std::uint32_t> colorBitsets;
        std::vector<Constraint1DMapper> mappers;
        Constraint1DSoa sequential;

    public:
        void preSolve();
        void solve(bool useBias, float timeStep);
        void pushBack(int count, entt::entity entity0, entt::entity entity1, TransformComponent& transform0, TransformComponent& transform1, RigidBodyDynamicComponent* dynamic0, RigidBodyDynamicComponent* dynamic1);
        void fillDefaults();
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
        unsigned char& flags;
        float& frequency;
        float& dampingRatio;
    };

    class Constraint1DViewer {
        int baseIndex;
        Constraint1DContainer& container;

    public:
        Constraint1DViewer(int baseIndex, Constraint1DContainer& container) : baseIndex(baseIndex), container(container) {}
         __forceinline Constraint1DView operator[] (int index) const {
            const int mapperIndex = baseIndex + index;
            auto& [color, i] = container.mappers[mapperIndex];
            return {
                color->linearBuffer[i],
                color->angular0Buffer[i],
                color->angular1Buffer[i],
                color->targetVelocityBuffer[i],
                color->cBuffer[i],
                color->minBuffer[i],
                color->maxBuffer[i],
                color->flagsBuffer[i],
                color->frequencyBuffer[i],
                color->dampingRatioBuffer[i]
            };
        }
    };
}

