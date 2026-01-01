#pragma once

#include <Constraint1D.h>
#include <Constraint1DW.h>
#include <entt.hpp>
#include <variant>

namespace physecs {

    template<int flags>
    class Constraint1DView {
        union {
            Constraint1DW<flags>* color;
            Constraint1D<flags>* sequential;
        };
        int offset;

    public:
        Constraint1DView(Constraint1DW<flags>& color, int offset) : color(&color), offset(offset) {}
        Constraint1DView(Constraint1D<flags>& sequential) : sequential(&sequential), offset(-1) {}

        __forceinline Constraint1DView& setLinear(const glm::vec3& linear) {
            if (offset < 0)
                sequential->n = linear;
            else
                color->linear.set(linear, offset);
            return *this;
        }

        __forceinline Constraint1DView& setAngular0(const glm::vec3& angular0) {
            if (offset < 0)
                sequential->r0xn = angular0;
            else
                color->angular0.set(angular0, offset);
            return *this;
        }

        __forceinline Constraint1DView& setAngular1(const glm::vec3& angular1) {
            if (offset < 0)
                sequential->r1xn = angular1;
            else
                color->angular1.set(angular1, offset);
            return *this;
        }

        __forceinline Constraint1DView& setTargetVelocity(float targetVelocity) {
            if (offset < 0)
                sequential->targetVelocity = targetVelocity;
            else
                color->targetVelocity.m128_f32[offset] = targetVelocity;
            return *this;
        }

        __forceinline Constraint1DView& setC(float c) {
            if (offset < 0)
                sequential->c = c;
            else
                color->c.m128_f32[offset] = c;
            return *this;
        }

        __forceinline Constraint1DView& setMin(float min) {
            if (offset < 0)
                sequential->min = min;
            else
                color->min.m128_f32[offset] = min;
            return *this;
        }

        __forceinline Constraint1DView& setMax(float max) {
            if (offset < 0)
                sequential->max = max;
            else
                color->max.m128_f32[offset] = max;
            return *this;
        }

        __forceinline Constraint1DView& setFrequency(float frequency) {
            if (offset < 0)
                sequential->frequency = frequency;
            else
                color->frequency.m128_f32[offset] = frequency;
            return *this;
        }

        __forceinline Constraint1DView& setDampingRatio(float dampingRatio) {
            if (offset < 0)
                sequential->dampingRatio = dampingRatio;
            else
                color->dampingRatio.m128_f32[offset] = dampingRatio;
            return *this;
        }
    };

    struct ConstraintRef {
        int baseIndex;
        int offset;
    };

    class Constraint1DContainer {
        template<template<int> typename Constraint, int flags>
        struct ConstraintList {
            std::vector<Constraint<flags>> constraints;
            __m128i lanes = _mm_setzero_si128();
        };

        template<template<int> typename Constraint, int... types>
        struct ConstraintCollection {
            std::tuple<ConstraintList<Constraint, types>...> constraints;

            template<int flags>
            ConstraintList<Constraint, flags>& get() {
                return std::get<ConstraintList<Constraint, flags>>(constraints);
            }
        };

        using SimdConstraints = ConstraintCollection<Constraint1DW, NONE, ANGULAR, SOFT, LIMITED, ANGULAR | SOFT, ANGULAR | LIMITED>;
        using OverflowConstraints = ConstraintCollection<Constraint1D, NONE, ANGULAR, SOFT, LIMITED, ANGULAR | SOFT, ANGULAR | LIMITED>;

        std::variant<SimdConstraints, OverflowConstraints> constraintCollection;

        std::vector<ConstraintRef> constraintRefs;

        friend class Constraint1DWriter;

    public:
        void preSolve();
        void solve(float timeStep);
        void clear();

        void setOverFlow() { constraintCollection.emplace<OverflowConstraints>(); }

        template<int flags>
        int createConstraint(RigidBodyDynamicComponent* dynamic0, RigidBodyDynamicComponent* dynamic1, int prevIndex) {
            if (std::holds_alternative<SimdConstraints>(constraintCollection)) {
                auto& constraintsCollection = std::get<SimdConstraints>(constraintCollection);
                auto& [constraintsList, lanes] = constraintsCollection.get<flags>();
                const auto shifted = _mm_slli_si128 (lanes, 4);
                auto cmp = _mm_and_epi32(_mm_cmpgt_epi32(shifted, lanes), _mm_cmpgt_epi32(lanes, _mm_set1_epi32(prevIndex)));
                cmp = _mm_shuffle_epi32(cmp, _MM_SHUFFLE(0, 1, 2, 3));
                const auto mask = _mm_movemask_ps(_mm_castsi128_ps(cmp));
                unsigned long i;
                const bool res = _BitScanForward(&i, mask);
                const int lane = res * (3 - i);
                int& currentIndex = lanes.m128i_i32[lane];
                if (currentIndex == constraintsList.size()) {
                    constraintsList.emplace_back();
                }
                constraintsList[currentIndex].dynamic0[lane] = dynamic0;
                constraintsList[currentIndex].dynamic1[lane] = dynamic1;
                constraintRefs.emplace_back(currentIndex, lane);
                return currentIndex++;
            }

            auto& constraintsCollection = std::get<OverflowConstraints>(constraintCollection);
            auto& constraintsList = constraintsCollection.get<flags>().constraints;
            constraintRefs.emplace_back(static_cast<int>(constraintsList.size()), -1);
            constraintsList.emplace_back(dynamic0, dynamic1);
            return 0;
        }
    };

    template<int... types>
    class FlagsMap {
        template<int flags>
        struct Entry {
            int value = -1;
        };

        std::tuple<Entry<types>...> entries;

    public:
        template<int flags>
        int& get() {
            return std::get<Entry<flags>>(entries).value;
        }
    };

    class Constraint1DLayout {
        Constraint1DContainer& container;
        RigidBodyDynamicComponent* dynamic0;
        RigidBodyDynamicComponent* dynamic1;
        FlagsMap<NONE, ANGULAR, SOFT, LIMITED, ANGULAR | SOFT, ANGULAR | LIMITED> currentIndices;

    public:
        Constraint1DLayout(Constraint1DContainer& container, RigidBodyDynamicComponent* dynamic0, RigidBodyDynamicComponent* dynamic1) :
        container(container),
        dynamic0(dynamic0),
        dynamic1(dynamic1) {}

        template<int flags = NONE, int count = 1>
        void createConstraints() {
            if constexpr (count) {
                currentIndices.get<flags>() = container.createConstraint<flags>(dynamic0, dynamic1, currentIndices.get<flags>());
                createConstraints<flags, count - 1>();
            }
        }
    };

    class Constraint1DWriter {
        Constraint1DContainer& container;
        int baseIndex;

    public:
        Constraint1DWriter(Constraint1DContainer& container, int index) : container(container), baseIndex(index) {}

        template<int flags = NONE>
        __forceinline Constraint1DView<flags> at(int offset) {
            auto& [i, o] = container.constraintRefs[baseIndex + offset];
            if (std::holds_alternative<Constraint1DContainer::SimdConstraints>(container.constraintCollection)) {
                auto& constraintsCollection = std::get<Constraint1DContainer::SimdConstraints>(container.constraintCollection);
                auto& constraintList = constraintsCollection.get<flags>();
                return Constraint1DView<flags>(constraintList.constraints[i], o);
            }

            auto& constraintsCollection = std::get<Constraint1DContainer::OverflowConstraints>(container.constraintCollection);
            auto& constraintList = constraintsCollection.get<flags>();
            return Constraint1DView<flags>(constraintList.constraints[i]);
        }
    };
}

