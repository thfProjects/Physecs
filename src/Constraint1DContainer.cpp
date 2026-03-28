#include "Constraint1DContainer.h"
#include "Constraint1D.cpp"
#include "Constraint1DW.cpp"
#include "SIMD.h"

void physecs::Constraint1DContainer::preSolve(const MassData* masses) {
    std::visit([masses](auto& constraintsCollection) {
        std::apply([&](auto&... constraintsLists) {
            (
                [&] {
                    for (auto& constraint : constraintsLists.constraints) {
                        constraint.preSolve(masses);
                    }
                }(),
                ...
            );
        }, constraintsCollection.constraints);
    }, constraintCollection);
}

void physecs::Constraint1DContainer::correctPositionError(PseudoVelocityData* pseudoVelocities) const {
    std::visit([pseudoVelocities](auto& constraintsCollection) {
        std::apply([&](auto&... constraintsLists) {
            (
                [&] {
                    for (auto& constraint : constraintsLists.constraints) {
                        constraint.correctPositionError(pseudoVelocities);
                    }
                }(),
                ...
            );
        }, constraintsCollection.constraints);
    }, constraintCollection);
}

void physecs::Constraint1DContainer::solve(VelocityData* velocities, float timeStep, bool warmStart) {
    std::visit([velocities, timeStep, warmStart](auto& constraintsCollection) {
        std::apply([&](auto&... constraintsLists) {
            (
                [&] {
                    for (auto& constraint : constraintsLists.constraints) {
                        constraint.solve(velocities, timeStep, warmStart);
                    }
                }(),
                ...
            );
        }, constraintsCollection.constraints);
    }, constraintCollection);
}

void physecs::Constraint1DContainer::clear() {
    std::visit([](auto& constraintsCollection) {
        std::apply([](auto&... constraintsList) {
            (
                [&] {
                    constraintsList.constraints.clear();
                    constraintsList.lanes = _mm_setzero_si128();
                }(),
                ...
            );
        }, constraintsCollection.constraints);
    }, constraintCollection);
    constraintRefs.clear();
}
