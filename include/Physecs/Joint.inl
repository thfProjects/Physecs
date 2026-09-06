#pragma once

#include "Joint.h"
#include "Constraint1DContainer.h"

namespace physecs {

template<typename Impl, typename Layout, typename Cache, typename Data>
JointSolverDesc JointImpl<Impl, Layout, Cache, Data>::getSolverDesc(entt::registry &registry, Constraint1DLayout &constraintLayout) {
    static_cast<Impl*>(this)->prepare(registry);
    createConstraints(Layout{}, constraintLayout);
    return JointSolverDesc{ &data, makeFinalConstraints };
}

template<typename Impl, typename Layout, typename Cache, typename Data>
void JointImpl<Impl, Layout, Cache, Data>::storeAccumulatedImpulses(Constraint1DReader &constraints) {
    storeAccumulatedImpulses(Layout{}, constraints);
}

template<typename Impl, typename Layout, typename Cache, typename Data>
template<typename Block>
__forceinline float* JointImpl<Impl, Layout, Cache, Data>::getLambdas() {
    if constexpr (Block::lambdas == nullptr) return nullptr;
    else if constexpr (std::is_array_v<std::remove_reference_t<decltype(cache.*Block::lambdas)>>) {
        return cache.*Block::lambdas;
    }
    else return &(cache.*Block::lambdas);
}

template<typename Impl, typename Layout, typename Cache, typename Data>
template<typename Block>
__forceinline void JointImpl<Impl, Layout, Cache, Data>::createConstraints(Constraint1DLayout& constraintLayout) {
    if constexpr (Block::gate != nullptr) if (!(data.*Block::gate)) return;
    constraintLayout.createConstraints<Block::flags, Block::count>(getLambdas<Block>());
}

template<typename Impl, typename Layout, typename Cache, typename Data>
template<typename... Blocks>
__forceinline void JointImpl<Impl, Layout, Cache, Data>::createConstraints(ConstraintLayout<Blocks...>, Constraint1DLayout& constraintLayout) {
    (createConstraints<Blocks>(constraintLayout), ...);
}

template<typename Impl, typename Layout, typename Cache, typename Data>
template<typename Block>
__forceinline void JointImpl<Impl, Layout, Cache, Data>::storeAccumulatedImpulses(Constraint1DReader& constraints) {
    if constexpr (Block::gate != nullptr) if (!(data.*Block::gate)) return;
    for (int i = 0; i < Block::count; ++i) {
        const float lambda = constraints.nextTotalLambda<Block::flags>();
        if constexpr (Block::lambdas != nullptr) getLambdas<Block>()[i] = lambda;
    }
}

template<typename Impl, typename Layout, typename Cache, typename Data>
template<typename... Blocks>
__forceinline void JointImpl<Impl, Layout, Cache, Data>::storeAccumulatedImpulses(ConstraintLayout<Blocks...>, Constraint1DReader& constraints) {
    (storeAccumulatedImpulses<Blocks>(constraints), ...);
}

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

template<typename Block, typename Data>
__forceinline void writeConstraints(const Data& data, const Constraint1DDescriptor* rows, int& row, Constraint1DWriter& constraints) {
    if constexpr (Block::gate != nullptr) if (!(data.*Block::gate)) return;
    for (int i = 0; i < Block::count; ++i) {
        const Constraint1DDescriptor& constraintRow = rows[row++];
        auto constraint = constraints.next<Block::flags>();
        if constexpr (!(Block::flags & ANGULAR)) {
            constraint
                .setLinear0(constraintRow.linear0)
                .setLinear1(constraintRow.linear1);
        }
        constraint
            .setAngular0(constraintRow.angular0)
            .setAngular1(constraintRow.angular1)
            .setC(constraintRow.geometricError)
            .setTargetVelocity(constraintRow.targetVelocity);
        if constexpr (Block::flags & LIMITED) constraint.setMin(constraintRow.minForce).setMax(constraintRow.maxForce);
        if constexpr (Block::flags & SOFT) constraint.setStiffness(constraintRow.stiffness).setDamping(constraintRow.damping);
    }
}

template<typename Data, typename... Blocks>
__forceinline void writeConstraints(ConstraintLayout<Blocks...>, const Data& data, const Constraint1DDescriptor* rows, Constraint1DWriter& constraints) {
    int row = 0;
    (writeConstraints<Blocks>(data, rows, row, constraints), ...);
}

template<int numAngularRows, int i, int j>
__forceinline float computeInvEffMassEntry(const Constraint1DDescriptor* constraintRows) {
    auto products = _mm_load_ps(glm::value_ptr(constraintRows[i].angular0)) * _mm_load_ps(glm::value_ptr(constraintRows[j].angular0))
        + _mm_load_ps(glm::value_ptr(constraintRows[i].angular1)) * _mm_load_ps(glm::value_ptr(constraintRows[j].angular1));
    if constexpr (i >= numAngularRows && j >= numAngularRows) {
        products += _mm_load_ps(glm::value_ptr(constraintRows[i].linear0)) * _mm_load_ps(glm::value_ptr(constraintRows[j].linear0))
            + _mm_load_ps(glm::value_ptr(constraintRows[i].linear1)) * _mm_load_ps(glm::value_ptr(constraintRows[j].linear1));
    }
    return sumXYZ(products);
}

template<typename F, int... Is>
__forceinline void repeatImpl(F&& f, std::integer_sequence<int, Is...>) {
    (f(std::integral_constant<int, Is>{}), ...);
}

template<int count, typename F>
__forceinline void repeat(F&& f) {
    repeatImpl(std::forward<F>(f), std::make_integer_sequence<int, count>{});
}

template<int numRows, int numAngularRows>
__forceinline void LDLtFactorize(const Constraint1DDescriptor* constraintRows, float L[][numRows], float* D) {
    // LDLt factorization of JM^-1J^T
    repeat<numRows>([&](auto I) [[msvc::forceinline]] {
        static constexpr int i = decltype(I)::value;
        float d = computeInvEffMassEntry<numAngularRows, i, i>(constraintRows);
        repeat<i>([&d, L, D](auto J) [[msvc::forceinline]] {
            static constexpr int j = decltype(J)::value;
            d -= L[j][i] * L[j][i] * D[j];
        });
        D[i] = d;
        const float effMass = 1.f / (d + 1e-8f);
        repeat<numRows-i-1>([effMass, constraintRows, L, D](auto J) [[msvc::forceinline]] {
            static constexpr int j = i + 1 + decltype(J)::value;
            float l = computeInvEffMassEntry<numAngularRows, j, i>(constraintRows);
            repeat<i>([&l, L, D](auto K) [[msvc::forceinline]] {
                static constexpr int k = decltype(K)::value;
                l -= D[k] * L[k][i] * L[k][j];
            });
            L[i][j] = l * effMass;
        });
    });
}

template<int numRows, int numAngularRows>
__forceinline void orthogonalize(Constraint1DDescriptor* constraintRows, float L[][numRows]) {
    // solve LJ' = J by forward substitution
    // J'M^-1J'^T will be a diagonal matrix, making gauss seidel for these constraints be identical to a block solve
    repeat<numRows - 1>([&](auto I) [[msvc::forceinline]] {
        static constexpr int i = 1 + decltype(I)::value;

        auto angular0 = _mm_load_ps(glm::value_ptr(constraintRows[i].angular0));
        auto angular1 = _mm_load_ps(glm::value_ptr(constraintRows[i].angular1));

        repeat<std::min(i, numAngularRows)>([&angular0, &angular1, constraintRows, L](auto J) [[msvc::forceinline]] {
            static constexpr int j = decltype(J)::value;
            auto Lji = _mm_set1_ps(L[j][i]);
            angular0 -= Lji * _mm_load_ps(glm::value_ptr(constraintRows[j].angular0));
            angular1 -= Lji * _mm_load_ps(glm::value_ptr(constraintRows[j].angular1));
        });

        if constexpr (i > numAngularRows) {
            auto linear0 = _mm_load_ps(glm::value_ptr(constraintRows[i].linear0));
            auto linear1 = _mm_load_ps(glm::value_ptr(constraintRows[i].linear1));

            repeat<i - numAngularRows>([&linear0, &linear1, &angular0, &angular1, constraintRows, L](auto J) [[msvc::forceinline]] {
                static constexpr int j = numAngularRows + decltype(J)::value;
                auto Lji = _mm_set1_ps(L[j][i]);
                linear0 -= Lji * _mm_load_ps(glm::value_ptr(constraintRows[j].linear0));
                linear1 -= Lji * _mm_load_ps(glm::value_ptr(constraintRows[j].linear1));
                angular0 -= Lji * _mm_load_ps(glm::value_ptr(constraintRows[j].angular0));
                angular1 -= Lji * _mm_load_ps(glm::value_ptr(constraintRows[j].angular1));
            });

            _mm_store_ps(glm::value_ptr(constraintRows[i].linear0), linear0);
            _mm_store_ps(glm::value_ptr(constraintRows[i].linear1), linear1);
        }

        _mm_store_ps(glm::value_ptr(constraintRows[i].angular0), angular0);
        _mm_store_ps(glm::value_ptr(constraintRows[i].angular1), angular1);
    });
}

template<typename Block, typename Data>
__forceinline void applyTransformAndMassScale(const Data& data, Constraint1DDescriptor* rows, int& row,
    const Mat3V& invR0, const Mat3V& invR1, const FloatW& sqrtInvMass0, const FloatW& sqrtInvMass1, const FloatW& sqrtInvInertia0, const FloatW& sqrtInvInertia1)
{
    if constexpr (Block::gate != nullptr) if (!(data.*Block::gate)) return;
    for (int i = 0; i < Block::count; ++i) {
        Constraint1DDescriptor& constraintRow = rows[row++];

        if constexpr (!(Block::flags & ANGULAR)) {
            _mm_store_ps(glm::value_ptr(constraintRow.linear0), _mm_load_ps(glm::value_ptr(constraintRow.linear0)) * sqrtInvMass0);
            _mm_store_ps(glm::value_ptr(constraintRow.linear1), _mm_load_ps(glm::value_ptr(constraintRow.linear1)) * sqrtInvMass1);
        }

        // blend back in the w lanes since they carry geometric error and target velocity
        const auto angular0 = _mm_load_ps(glm::value_ptr(constraintRow.angular0));
        _mm_store_ps(glm::value_ptr(constraintRow.angular0), _mm_blend_ps((invR0 * angular0) * sqrtInvInertia0, angular0, 0x8));

        const auto angular1 = _mm_load_ps(glm::value_ptr(constraintRow.angular1));
        _mm_store_ps(glm::value_ptr(constraintRow.angular1), _mm_blend_ps((invR1 * angular1) * sqrtInvInertia1, angular1, 0x8));
    }
}

template<typename Data, typename... Blocks>
__forceinline void applyTransformAndMassScale(ConstraintLayout<Blocks...>, const Data& data, Constraint1DDescriptor* rows, const Constraint1DWriterContext& context) {
    const FloatW one = _mm_set1_ps(1.f);

    const FloatW sqrtInvInertia0 = _mm_load_ps(glm::value_ptr(context.massData0->sqrtInvInertia));
    const FloatW sqrtInvInertia1 = _mm_load_ps(glm::value_ptr(context.massData1->sqrtInvInertia));

    // sqrtInvMass is in lane 4 of sqrtInvInertia, leave lane 4 on sqrtInvMass vector as 1 to avoid multiplying stiffness and damping
    const FloatW sqrtInvMass0 = _mm_blend_ps(_mm_shuffle_ps(sqrtInvInertia0, sqrtInvInertia0, _MM_SHUFFLE(3,3,3,3)), one, 0x8);
    const FloatW sqrtInvMass1 = _mm_blend_ps(_mm_shuffle_ps(sqrtInvInertia1, sqrtInvInertia1, _MM_SHUFFLE(3,3,3,3)), one, 0x8);

    int row = 0;
    (applyTransformAndMassScale<Blocks>(data, rows, row, *context.invR0, *context.invR1, sqrtInvMass0, sqrtInvMass1, sqrtInvInertia0, sqrtInvInertia1), ...);
}

template<typename Impl, typename Layout, typename Cache, typename Data>
void JointImpl<Impl, Layout, Cache, Data>::makeFinalConstraints(const JointWorldSpaceData& worldSpaceData, void* additionalData, const Constraint1DWriterContext& context, Constraint1DWriter& constraints) {
    Constraint1DDescriptor constraintRows[Layout::count];

    const FloatW zero = _mm_setzero_ps();
    for (int i = 0; i < Layout::count; ++i) {
        Constraint1DDescriptor& constraintRow = constraintRows[i];
        _mm_store_ps(glm::value_ptr(constraintRow.linear0), zero);
        _mm_store_ps(glm::value_ptr(constraintRow.linear1), zero);
        _mm_store_ps(glm::value_ptr(constraintRow.angular0), zero);
        _mm_store_ps(glm::value_ptr(constraintRow.angular1), zero);
    }

    Impl::makeConstraints(worldSpaceData, additionalData, constraintRows);

    const Data& data = *static_cast<const Data*>(additionalData);

    applyTransformAndMassScale(Layout{}, data, constraintRows, context);

    constexpr int n = Layout::hardEqualityCount;
    if constexpr (n > 1) {
        float L[n][n]; // column major lower triangular
        float D[n]; // diagonal

        constexpr int numAngular = Layout::hardEqualityAngularCount;
        LDLtFactorize<n, numAngular>(constraintRows, L, D);
        orthogonalize<n, numAngular>(constraintRows, L);
    }

    writeConstraints(Layout{}, data, constraintRows, constraints);
}

}
