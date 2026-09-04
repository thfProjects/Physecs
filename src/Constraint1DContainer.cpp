#include "Constraint1DContainer.h"
#include "Constraint1D.cpp"
#include "Constraint1DW.cpp"
#include "SIMD.h"

template<typename F>
_forceinline void physecs::Constraint1DContainer::visit(F&& f) {
    if (isOverflow) f(overflowConstraints);
    else f(simdConstraints);
}

template<typename F>
_forceinline void physecs::Constraint1DContainer::forEachList(F&& f) {
    visit([&f](auto& constraintsCollection) {
        std::apply([&f](auto&... constraintsLists) {
            (f(constraintsLists), ...);
        }, constraintsCollection.constraints);
    });
}

template<typename F>
_forceinline void physecs::Constraint1DContainer::forEachConstraint(F&& f) {
    forEachList([&f](auto& constraintsList) {
        for (auto& constraint : constraintsList.constraints) {
            f(constraint);
        }
    });
}

void physecs::Constraint1DContainer::preSolve(VelocityData* velocities, PseudoVelocityData* pseudoVelocities) {
    forEachConstraint([=](auto& constraint) [[msvs::forceinline]] {
        constraint.preSolve(velocities, pseudoVelocities);
    });
}

void physecs::Constraint1DContainer::solve(VelocityData* velocities, float timeStep, bool useBias) {
    forEachConstraint([=](auto& constraint) [[msvs::forceinline]] {
        constraint.solve(velocities, timeStep, useBias);
    });
}

void physecs::Constraint1DContainer::clear() {
    forEachList([](auto& constraintsList) [[msvs::forceinline]] {
        constraintsList.constraints.clear();
        constraintsList.lanes = _mm_setzero_si128();
    });
    constraintRefs.clear();
}
