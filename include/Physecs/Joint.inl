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
__forceinline float* JointImpl<Impl, Layout, Cache, Data>::getAccumulatedImpulses() {
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
    constraintLayout.createConstraints<Block::flags, Block::count>(getAccumulatedImpulses<Block>());
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
        if constexpr (Block::lambdas != nullptr) getAccumulatedImpulses<Block>()[i] = lambda;
    }
}

template<typename Impl, typename Layout, typename Cache, typename Data>
template<typename... Blocks>
__forceinline void JointImpl<Impl, Layout, Cache, Data>::storeAccumulatedImpulses(ConstraintLayout<Blocks...>, Constraint1DReader& constraints) {
    (storeAccumulatedImpulses<Blocks>(constraints), ...);
}

template<typename Block, typename Data>
__forceinline void writeConstraints(const Data& data, const Constraint1DDescriptor* rows, float* effMasses, int& row, Constraint1DWriter& constraints) {
    if constexpr (Block::gate != nullptr) if (!(data.*Block::gate)) return;
    for (int i = 0; i < Block::count; ++i) {
        const int idx = row++;
        const Constraint1DDescriptor& constraintRow = rows[idx];
        constraints.writeNext<Block::flags>(constraintRow, effMasses[idx]);
    }
}

template<typename Data, typename... Blocks>
__forceinline void writeConstraints(ConstraintLayout<Blocks...>, const Data& data, const Constraint1DDescriptor* rows, float* effMasses, Constraint1DWriter& constraints) {
    int row = 0;
    (writeConstraints<Blocks>(data, rows, effMasses, row, constraints), ...);
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
__forceinline void LUFactorize(const Constraint1DDescriptor* constraintRows, float L[][numRows], float* effMasses) {
    constexpr float invMassScale = 1.01f;
    // LU factorization of JM^-1J^T
    repeat<numRows>([&](auto I) [[msvc::forceinline]] {
        static constexpr int i = decltype(I)::value;
        float d = computeInvEffMassEntry<numAngularRows, i, i>(constraintRows) * invMassScale + 1e-8f;
        repeat<i>([&d, L](auto J) [[msvc::forceinline]] {
            static constexpr int j = decltype(J)::value;
            d -= L[i][j] * L[j][i];
        });
        const float effMass = 1.f / d;
        effMasses[i] = effMass;
        repeat<numRows-i-1>([effMass, constraintRows, L](auto J) [[msvc::forceinline]] {
            static constexpr int j = i + 1 + decltype(J)::value;
            float l = computeInvEffMassEntry<numAngularRows, j, i>(constraintRows);
            repeat<i>([&l, L](auto K) [[msvc::forceinline]] {
                static constexpr int k = decltype(K)::value;
                l -= L[i][k] * L[k][j];
            });
            L[i][j] = l * effMass;
            L[j][i] = l;
        });
    });
}

template<int numRows, int numAngularRows>
__forceinline void orthogonalize(Constraint1DDescriptor* constraintRows, float L[][numRows]) {
    // solve LJ' = J by forward substitution
    // J'M^-1J'^T will be a diagonal matrix, making gauss seidel for these constraints be identical to a block solve
    repeat<numRows - 1>([&](auto I) [[msvc::forceinline]] {
        static constexpr int i = 1 + decltype(I)::value;

        FloatW linear0, linear1;
        if constexpr (i > numAngularRows) {
            linear0 = _mm_load_ps(glm::value_ptr(constraintRows[i].linear0));
            linear1 = _mm_load_ps(glm::value_ptr(constraintRows[i].linear1));
        }

        FloatW angular0 = _mm_load_ps(glm::value_ptr(constraintRows[i].angular0));
        FloatW angular1 = _mm_load_ps(glm::value_ptr(constraintRows[i].angular1));

        repeat<i>([&linear0, &linear1, &angular0, &angular1, constraintRows, L](auto J) [[msvc::forceinline]] {
            static constexpr int j = decltype(J)::value;
            const FloatW Lji = _mm_set1_ps(L[j][i]);
            if constexpr (j >= numAngularRows) {
                linear0 -= Lji * _mm_load_ps(glm::value_ptr(constraintRows[j].linear0));
                linear1 -= Lji * _mm_load_ps(glm::value_ptr(constraintRows[j].linear1));
            }
            angular0 -= Lji * _mm_load_ps(glm::value_ptr(constraintRows[j].angular0));
            angular1 -= Lji * _mm_load_ps(glm::value_ptr(constraintRows[j].angular1));
        });

        if constexpr (i > numAngularRows) {
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
    float effMasses[n];
    if constexpr (n > 0) {
        float L[n][n]; // column major lower triangular L and upper triangular DL^T

        constexpr int numAngular = Layout::hardEqualityAngularCount;
        LUFactorize<n, numAngular>(constraintRows, L, effMasses);
        orthogonalize<n, numAngular>(constraintRows, L);
    }

    writeConstraints(Layout{}, data, constraintRows, effMasses, constraints);
}

}
