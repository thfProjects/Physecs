#include "Constraint1DContainer.h"
#include "Transform.h"
#include <Constraint1D.h>
#include "SIMD.h"

void physecs::Constraint1DContainer::preSolve() {
    for (auto& constraintColor : graphColors) {
        for (auto& constraintsW : constraintColor.constraints) {
            constraintsW.preSolve();
        }
    }
    for (auto& constraint : sequential) {
        constraint.prepare();
        if (constraint.flags & Constraint1D::SOFT || glm::abs(constraint.c) > 1e-4 || glm::abs(constraint.totalLambda) > 10000) continue;
        constraint.warmStart();
    }
}

void physecs::Constraint1DContainer::solve(bool useBias, float timeStep) {
    for (auto& constraintColor : graphColors) {
        for (auto& constraintsW : constraintColor.constraints) {
            constraintsW.solve(timeStep);
        }
    }
    for (auto& constraint : sequential) {
        constraint.solve(useBias, timeStep);
    }
}

void physecs::Constraint1DContainer::pushBack(int count, entt::entity entity0, entt::entity entity1, TransformComponent &transform0, TransformComponent &transform1, RigidBodyDynamicComponent *dynamic0, RigidBodyDynamicComponent *dynamic1) {
    auto& colors0 = colorBitsets[entity0];
    auto& colors1 = colorBitsets[entity1];
    for (int j = 0; j < count; ++j) {
        const auto colorsUnion = colors0 | colors1;
        unsigned long i;
        if (_BitScanForward(&i, ~colorsUnion)) {
            colors0 |= 1 << i;
            colors1 |= 1 << i;

            auto& [constraints, size] = graphColors[i];
            if (size == constraints.size() * 4) {
                constraints.emplace_back();
            }
            const int baseIndex = constraints.size() - 1;
            const int offset = size - baseIndex * 4;
            views.emplace_back(&constraints, baseIndex, offset);
            constraints[baseIndex].transform0[offset] = &transform0;
            constraints[baseIndex].transform1[offset] = &transform1;
            constraints[baseIndex].dynamic0[offset] = dynamic0;
            constraints[baseIndex].dynamic1[offset] = dynamic1;
            ++size;
        }
        else {
            views.emplace_back(&sequential, sequential.size());
            sequential.emplace_back(transform0, transform1, dynamic0, dynamic1, glm::vec3(0), glm::vec3(0), glm::vec3(0), 0, 0, std::numeric_limits<float>::lowest(), std::numeric_limits<float>::max());
        }
    }
}

void physecs::Constraint1DContainer::clear() {
    for (auto& color : graphColors) {
        color.constraints.clear();
        color.size = 0;
    }
    sequential.clear();
    views.clear();
    colorBitsets.clear();
}
