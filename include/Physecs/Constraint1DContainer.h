#pragma once

#include <Constraint1D.h>
#include <Constraint1DW.h>
#include <entt.hpp>

namespace physecs {
    struct Constraint1D;
}

namespace physecs {
    struct Constraint1DW;
}

namespace physecs {

    struct GraphColor {
        std::vector<Constraint1DW> constraints;
        int size = 0;
    };

    class Constraint1DView {
        union {
            std::vector<Constraint1DW>* color;
            std::vector<Constraint1D>* sequential;
        };
        int i;
        int offset;

    public:
        Constraint1DView(std::vector<Constraint1DW>* color, int index, int offset) : color(color), i(index), offset(offset) {}
        Constraint1DView(std::vector<Constraint1D>* sequential, int index) : sequential(sequential), i(index), offset(-1) {}

        __forceinline Constraint1DView& setLinear(const glm::vec3& linear) {
            if (offset < 0)
                (*sequential)[i].n = linear;
            else
                (*color)[i].linear.set(linear, offset);
            return *this;
        }

        __forceinline Constraint1DView& setAngular0(const glm::vec3& angular0) {
            if (offset < 0)
                (*sequential)[i].r0xn = angular0;
            else
                (*color)[i].angular0.set(angular0, offset);
            return *this;
        }

        __forceinline Constraint1DView& setAngular1(const glm::vec3& angular1) {
            if (offset < 0)
                (*sequential)[i].r1xn = angular1;
            else
                (*color)[i].angular1.set(angular1, offset);
            return *this;
        }

        __forceinline Constraint1DView& setTargetVelocity(float targetVelocity) {
            if (offset < 0)
                (*sequential)[i].targetVelocity = targetVelocity;
            else
                (*color)[i].targetVelocity.m128_f32[offset] = targetVelocity;
            return *this;
        }

        __forceinline Constraint1DView& setC(float c) {
            if (offset < 0)
                (*sequential)[i].c = c;
            else
                (*color)[i].c.m128_f32[offset] = c;
            return *this;
        }

        __forceinline Constraint1DView& setMin(float min) {
            if (offset < 0)
                (*sequential)[i].min = min;
            else
                (*color)[i].min.m128_f32[offset] = min;
            return *this;
        }

        __forceinline Constraint1DView& setMax(float max) {
            if (offset < 0)
                (*sequential)[i].max = max;
            else
                (*color)[i].max.m128_f32[offset] = max;
            return *this;
        }

        __forceinline Constraint1DView& setFlags(unsigned char flags) {
            if (offset < 0)
                (*sequential)[i].flags = flags;
            else
                (*color)[i].flags[offset] = flags;
            return *this;
        }

        __forceinline Constraint1DView& setFrequency(float frequency) {
            if (offset < 0)
                (*sequential)[i].frequency = frequency;
            else
                (*color)[i].frequency.m128_f32[offset] = frequency;
            return *this;
        }

        __forceinline Constraint1DView& setDampingRatio(float dampingRatio) {
            if (offset < 0)
                (*sequential)[i].dampingRatio = dampingRatio;
            else
                (*color)[i].dampingRatio.m128_f32[offset] = dampingRatio;
            return *this;
        }
    };

    class Constraint1DContainer {
        constexpr static int maxColors = 32;

        GraphColor graphColors[maxColors];
        entt::dense_map<entt::entity, std::uint32_t> colorBitsets;
        std::vector<Constraint1DView> views;
        std::vector<Constraint1D> sequential;

    public:
        void preSolve();
        void solve(bool useBias, float timeStep);
        void pushBack(int count, entt::entity entity0, entt::entity entity1, TransformComponent& transform0, TransformComponent& transform1, RigidBodyDynamicComponent* dynamic0, RigidBodyDynamicComponent* dynamic1);
        void clear();
        Constraint1DView* getView(int index) { return &views[index]; }
    };
}

