#include "Constraint1DContainer.h"
#include "Constraint1D.cpp"
#include "Constraint1DW.cpp"
#include "SIMD.h"

void physecs::Constraint1DContainer::preSolve(const MassData* masses, VelocityData* velocities, PseudoVelocityData* pseudoVelocities) {
    std::visit([masses, velocities, pseudoVelocities](auto& constraintsCollection) {
        std::apply([&](auto&... constraintsLists) {
            (
                [&] {
                    for (auto& constraint : constraintsLists.constraints) {
                        constraint.preSolve(masses, velocities, pseudoVelocities);
                    }
                }(),
                ...
            );
        }, constraintsCollection.constraints);
    }, constraintCollection);
}

void physecs::Constraint1DContainer::solve(VelocityData* velocities, float timeStep, bool useBias) {
    std::visit([velocities, timeStep, useBias](auto& constraintsCollection) {
        std::apply([&](auto&... constraintsLists) {
            (
                [&] {
                    for (auto& constraint : constraintsLists.constraints) {
                        constraint.solve(velocities, timeStep, useBias);
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
