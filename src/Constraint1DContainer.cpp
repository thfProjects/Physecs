#include "Constraint1DContainer.h"
#include "Constraint1D.cpp"
#include "Constraint1DW.cpp"
#include "SIMD.h"

void physecs::Constraint1DContainer::preSolve() {
    std::visit([](auto& constraintsCollection) {
        std::apply([](auto&... constraintsLists) {
            (
                [&] {
                    for (auto& constraint : constraintsLists.constraints) {
                        constraint.preSolve();
                    }
                }(),
                ...
            );
        }, constraintsCollection.constraints);
    }, constraintCollection);
}

void physecs::Constraint1DContainer::solve(float timeStep) {
    std::visit([timeStep](auto& constraintsCollection) {
        std::apply([&](auto&... constraintsLists) {
            (
                [&] {
                    for (auto& constraint : constraintsLists.constraints) {
                        constraint.solve(timeStep);
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
