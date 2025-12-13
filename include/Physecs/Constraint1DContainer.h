#pragma once

#include <entt.hpp>

#include "Components.h"

struct TransformComponent;

namespace physecs {

    struct Vec3W;

    class Constraint1DSoa {

        friend class Constraint1DView;

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
        Field<Vec3W> velocity0Buffer;
        Field<Vec3W> velocity1Buffer;
        Field<Vec3W> angularVelocity0Buffer;
        Field<Vec3W> angularVelocity1Buffer;
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

    class Constraint1DView {
        Constraint1DSoa* soa;
        int i;

    public:
        Constraint1DView(Constraint1DSoa* soa, int index) : soa(soa), i(index) {}

        __forceinline Constraint1DView& setLinear(const glm::vec3& n) {
            soa->linearBuffer[i] = n;
            return *this;
        }

        __forceinline Constraint1DView& setAngular0(const glm::vec3& angular0) {
            soa->angular0Buffer[i] = angular0;
            return *this;
        }

        __forceinline Constraint1DView& setAngular1(const glm::vec3& angular1) {
            soa->angular1Buffer[i] = angular1;
            return *this;
        }

        __forceinline Constraint1DView& setTargetVelocity(float targetVelocity) {
            soa->targetVelocityBuffer[i] = targetVelocity;
            return *this;
        }

        __forceinline Constraint1DView& setC(float c) {
            soa->cBuffer[i] = c;
            return *this;
        }

        __forceinline Constraint1DView& setMin(float min) {
            soa->minBuffer[i] = min;
            return *this;
        }

        __forceinline Constraint1DView& setMax(float max) {
            soa->maxBuffer[i] = max;
            return *this;
        }

        __forceinline Constraint1DView& setFlags(unsigned char flags) {
            soa->flagsBuffer[i] = flags;
            return *this;
        }

        __forceinline Constraint1DView& setFrequency(float frequency) {
            soa->frequencyBuffer[i] = frequency;
            return *this;
        }

        __forceinline Constraint1DView& setDampingRatio(float dampingRatio) {
            soa->dampingRatioBuffer[i] = dampingRatio;
            return *this;
        }
    };

    class Constraint1DContainer {
        constexpr static int maxColors = 32;

        Constraint1DSoa constraintColors[maxColors];
        entt::dense_map<entt::entity, std::uint32_t> colorBitsets;
        std::vector<Constraint1DView> views;
        Constraint1DSoa sequential;

    public:
        void preSolve();
        void solve(bool useBias, float timeStep);
        void pushBack(int count, entt::entity entity0, entt::entity entity1, TransformComponent& transform0, TransformComponent& transform1, RigidBodyDynamicComponent* dynamic0, RigidBodyDynamicComponent* dynamic1);
        void fillDefaults();
        void clear();
        Constraint1DView* getView(int index) { return &views[index]; }
    };
}

