#pragma once

#include <Constraint1D.h>
#include <Constraint1DW.h>
#include <entt.hpp>
#include <MathUtil.h>
#include <variant>
#include <Constraint1DFlags.h>

namespace physecs {

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

        template<template<int> typename Constraint>
        using Collection = ConstraintCollection<Constraint, NONE, ANGULAR, SOFT, LIMITED, ANGULAR | SOFT, ANGULAR | LIMITED>;

        using SimdConstraints = Collection<Constraint1DW>;
        using OverflowConstraints = Collection<Constraint1D>;

        union {
            SimdConstraints simdConstraints;
            OverflowConstraints overflowConstraints;
        };

        std::vector<ConstraintRef> constraintRefs;

        bool isOverflow = false;

        template<typename F>
        void visit(F&& f);

        template<typename F>
        void forEachList(F&& f);

        template<typename F>
        void forEachConstraint(F&& f);

        friend class Constraint1DLayout;
        friend class Constraint1DWriter;
        friend class Constraint1DReader;

    public:
        Constraint1DContainer() {
            std::construct_at(&simdConstraints);
        }

        ~Constraint1DContainer() {
            if (isOverflow) std::destroy_at(&overflowConstraints);
            else std::destroy_at(&simdConstraints);
        }
        void preSolve(VelocityData* velocities, PseudoVelocityData* pseudoVelocities, float timeStep);
        void solve(VelocityData* velocities, float baumgarteFactor);
        void clear();

        void setOverFlow() {
            isOverflow = true;
            std::destroy_at(&simdConstraints);
            std::construct_at(&overflowConstraints);
        }

        template<int flags>
        int createConstraint(int bodyIndex0, int bodyIndex1, float initLambda, int prevIndex) {
            if (!isOverflow) {
                auto& [constraintsList, lanes] = simdConstraints.get<flags>();
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
                constraintsList[currentIndex].bodies0[lane] = bodyIndex0;
                constraintsList[currentIndex].bodies1[lane] = bodyIndex1;
                constraintsList[currentIndex].totalLambda.m128_f32[lane] = initLambda;
                constraintRefs.emplace_back(currentIndex, lane);
                return currentIndex++;
            }

            auto& constraintsList = overflowConstraints.get<flags>().constraints;
            constraintRefs.emplace_back(static_cast<int>(constraintsList.size()), -1);
            constraintsList.emplace_back(bodyIndex0, bodyIndex1, initLambda);
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
        int b0;
        int b1;
        FlagsMap<NONE, ANGULAR, SOFT, LIMITED, ANGULAR | SOFT, ANGULAR | LIMITED> currentIndices;

    public:
        Constraint1DLayout(Constraint1DContainer& container, int b0, int b1) :
        container(container),
        b0(b0),
        b1(b1) {}

        template<int flags = NONE, int count = 1>
        void createConstraints(const float* initLambda = nullptr) {
            if constexpr (count) {
                currentIndices.get<flags>() = container.createConstraint<flags>(b0, b1, initLambda ? *initLambda : 0.f, currentIndices.get<flags>());
                createConstraints<flags, count - 1>(initLambda ? initLambda + 1 : nullptr);
            }
        }
    };

    class Constraint1DReader {
        Constraint1DContainer& container;
        int index = 0;

    public:
        Constraint1DReader(Constraint1DContainer& container) : container(container) {}

        template<int flags = NONE>
        __forceinline float nextTotalLambda() {
            auto& [i, o] = container.constraintRefs[index++];
            if (!container.isOverflow) {
                auto& constraintsList = container.simdConstraints.get<flags>();
                return constraintsList.constraints[i].totalLambda.m128_f32[o];
            }
            auto& constraintsList = container.overflowConstraints.get<flags>();
            return constraintsList.constraints[i].totalLambda;
        }
    };

    struct Constraint1DDescriptor {
        alignas(16) glm::vec3 linear0;
        union {
            float stiffness;
            float minForce;
        };
        alignas(16) glm::vec3 linear1;
        union {
            float damping;
            float maxForce;
        };
        alignas(16) glm::vec3 angular0;
        float targetVelocity;
        alignas(16) glm::vec3 angular1;
        float geometricError;
    };

    template<bool isOverflow, int flags>
    class Constraint1DView {
        using ConstraintT = std::conditional_t<isOverflow, Constraint1D<flags>, Constraint1DW<flags>>;
        ConstraintT* constraint;
        int offset;

    public:
        Constraint1DView(ConstraintT& constraint, int offset) : constraint(&constraint), offset(offset) {}

        __forceinline Constraint1DView& setLinear0(const glm::vec3& linear0) {
            if constexpr (isOverflow) {
                constraint->linear0 = linear0;
            }
            else {
                constraint->linear0.set(linear0, offset);
            }
            return *this;
        }

        __forceinline Constraint1DView& setLinear1(const glm::vec3& linear1) {
            if constexpr (isOverflow) {
                constraint->linear1 = linear1;
            }
            else {
                constraint->linear1.set(linear1, offset);
            }
            return *this;
        }

        __forceinline Constraint1DView& setAngular0(const glm::vec3& angular0) {
            if constexpr (isOverflow)
                constraint->angular0 = angular0;
            else
                constraint->angular0.set(angular0, offset);
            return *this;
        }

        __forceinline Constraint1DView& setAngular1(const glm::vec3& angular1) {
            if constexpr (isOverflow)
                constraint->angular1 = angular1;
            else
                constraint->angular1.set(angular1, offset);
            return *this;
        }

        __forceinline Constraint1DView& setTargetVelocity(float targetVelocity) {
            if constexpr (isOverflow)
                constraint->targetVelocity = targetVelocity;
            else
                constraint->targetVelocity.m128_f32[offset] = targetVelocity;
            return *this;
        }

        __forceinline Constraint1DView& setC(float c) {
            if constexpr (isOverflow)
                constraint->c = c;
            else
                constraint->c.m128_f32[offset] = c;
            return *this;
        }

        __forceinline Constraint1DView& setMin(float min) {
            if constexpr (isOverflow)
                constraint->min = min;
            else
                constraint->min.m128_f32[offset] = min;
            return *this;
        }

        __forceinline Constraint1DView& setMax(float max) {
            if constexpr (isOverflow)
                constraint->max = max;
            else
                constraint->max.m128_f32[offset] = max;
            return *this;
        }

        __forceinline Constraint1DView& setStiffness(float stiffness) {
            if constexpr (isOverflow)
                constraint->springParams.stiffness = stiffness;
            else
                constraint->springParams.stiffness.m128_f32[offset] = stiffness;
            return *this;
        }

        __forceinline Constraint1DView& setDamping(float damping) {
            if constexpr (isOverflow)
                constraint->springParams.damping = damping;
            else
                constraint->springParams.damping.m128_f32[offset] = damping;
            return *this;
        }
    };

    class Constraint1DWriter {
        Constraint1DContainer& container;
        int index = 0;

        template<bool isOverflow>
        using ConstraintCollectionT = std::conditional_t<isOverflow, Constraint1DContainer::OverflowConstraints, Constraint1DContainer::SimdConstraints>;

        template<bool isOverflow, int flags>
        void writeNextImpl(ConstraintCollectionT<isOverflow>& constraintCollection, const Constraint1DDescriptor& row) {
            auto& [i, o] = container.constraintRefs[index++];
            auto& constraintsList = constraintCollection.template get<flags>();
            auto& constraint = constraintsList.constraints[i];
            auto constraintView = Constraint1DView<isOverflow, flags>(constraint, o);
            if constexpr (!(flags & ANGULAR)) {
                constraintView
                    .setLinear0(row.linear0)
                    .setLinear1(row.linear1);
            }
            constraintView
                .setAngular0(row.angular0)
                .setAngular1(row.angular1)
                .setC(row.geometricError)
                .setTargetVelocity(row.targetVelocity);
            if constexpr (flags & LIMITED) constraintView.setMin(row.minForce).setMax(row.maxForce);
            if constexpr (flags & SOFT) constraintView.setStiffness(row.stiffness).setDamping(row.damping);
        }

    public:
        Constraint1DWriter(Constraint1DContainer& container) : container(container) {}

        template<int flags>
        _forceinline void writeNext(const Constraint1DDescriptor& row) {
            if (!container.isOverflow) writeNextImpl<false, flags>(container.simdConstraints, row);
            else writeNextImpl<true, flags>(container.overflowConstraints, row);
        }
    };
}

