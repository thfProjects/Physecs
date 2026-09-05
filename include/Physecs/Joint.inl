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

template<typename Impl, typename Layout, typename Cache, typename Data>
template<typename Block>
__forceinline void JointImpl<Impl, Layout, Cache, Data>::writeConstraints(const Data& data, const Constraint1DDescriptor* rows, int& row, Constraint1DWriter& constraints) {
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

template<typename Impl, typename Layout, typename Cache, typename Data>
template<typename... Blocks>
__forceinline void JointImpl<Impl, Layout, Cache, Data>::writeConstraints(ConstraintLayout<Blocks...>, const Data& data, const Constraint1DDescriptor* rows, Constraint1DWriter& constraints) {
    int row = 0;
    (writeConstraints<Blocks>(data, rows, row, constraints), ...);
}

template<int numAngularRows>
float computeInvEffMassEntry(const Constraint1DDescriptor* constraintRows, int i, int j) {
    float entry = glm::dot(constraintRows[i].angular0, constraintRows[j].angular0)
        + glm::dot(constraintRows[i].angular1, constraintRows[j].angular1);
    if (i >= numAngularRows && j >= numAngularRows) {
        entry += glm::dot(constraintRows[i].linear0, constraintRows[j].linear0)
            + glm::dot(constraintRows[i].linear1, constraintRows[j].linear1);
    }
    return entry;
}

template<int numRows, int numAngularRows>
void LDLtFactorize(const Constraint1DDescriptor* constraintRows, float L[][numRows], float* D) {
    // LDLt factorization of JM^-1J^T
    for (int i = 0; i < numRows; ++i) {
        D[i] = computeInvEffMassEntry<numAngularRows>(constraintRows, i, i);
        for (int j = 0; j < i; ++j) {
            D[i] -= L[j][i] * L[j][i] * D[j];
        }
        const float effMass = 1.f / (D[i] + 1e-8f);
        for (int j = i + 1; j < numRows; ++j) {
            L[i][j] = computeInvEffMassEntry<numAngularRows>(constraintRows, j, i);
            for (int k = 0; k < i; ++k) {
                L[i][j] -= D[k] * L[k][i] * L[k][j];
            }
            L[i][j] *= effMass;
        }
    }
}

template<int numRows, int numAngularRows>
void orthogonalize(Constraint1DDescriptor* constraintRows, float L[][numRows]) {
    // solve LJ' = J by forward substitution
    // J'M^-1J'^T will be a diagonal matrix, making gauss seidel for these constraints be identical to a block solve
    for (int i = 0; i < numRows; ++i) {
        for (int j = 0; j < i; ++j) {
            if (j >= numAngularRows) {
                constraintRows[i].linear0 -= L[j][i] * constraintRows[j].linear0;
                constraintRows[i].linear1 -= L[j][i] * constraintRows[j].linear1;
            }
            constraintRows[i].angular0 -= L[j][i] * constraintRows[j].angular0;
            constraintRows[i].angular1 -= L[j][i] * constraintRows[j].angular1;
            constraintRows[i].geometricError -= L[j][i] * constraintRows[j].geometricError;
        }
    }
}

template<typename Impl, typename Layout, typename Cache, typename Data>
void JointImpl<Impl, Layout, Cache, Data>::makeFinalConstraints(const JointWorldSpaceData& worldSpaceData, void* additionalData, const Constraint1DWriterContext& context, Constraint1DWriter& constraints) {
    Constraint1DDescriptor constraintRows[Layout::count];
    Impl::makeConstraints(worldSpaceData, additionalData, constraintRows);

    // apply transform and mass scale
    for (int i = 0; i < Layout::count; ++i) {
        constraintRows[i].linear0 *= context.massData0->sqrtInvMass;
        constraintRows[i].linear1 *= context.massData1->sqrtInvMass;
        constraintRows[i].angular0 = multiplyTranspose(*context.r0, constraintRows[i].angular0) * context.massData0->sqrtInvInertia;
        constraintRows[i].angular1 = multiplyTranspose(*context.r1, constraintRows[i].angular1) * context.massData1->sqrtInvInertia;
    }

    constexpr int n = Layout::hardEqualityCount;
    if constexpr (n > 1) {
        float L[n][n]; // column major lower triangular
        float D[n]; // diagonal

        constexpr int numAngular = Layout::hardEqualityAngularCount;
        LDLtFactorize<n, numAngular>(constraintRows, L, D);
        orthogonalize<n, numAngular>(constraintRows, L);
    }

    writeConstraints(Layout{}, *static_cast<const Data*>(additionalData), constraintRows, constraints);
}

}
