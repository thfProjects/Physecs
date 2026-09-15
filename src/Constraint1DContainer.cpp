#include "Constraint1DContainer.h"
#include "Constraint1D.cpp"
#include "Constraint1DW.cpp"
#include "SIMD.h"

template<typename F>
PHYSECS_FORCE_INLINE void physecs::Constraint1DContainer::visit(F&& f) {
    if (isOverflow) f(overflowConstraints);
    else f(simdConstraints);
}

template<typename F>
PHYSECS_FORCE_INLINE void physecs::Constraint1DContainer::forEachList(F&& f) {
    visit([&f](auto& constraintsCollection) PHYSECS_FORCE_INLINE_LAMBDA {
        std::apply([&f](auto&... constraintsLists) PHYSECS_FORCE_INLINE_LAMBDA {
            (f(constraintsLists), ...);
        }, constraintsCollection.constraints);
    });
}

template<typename F>
PHYSECS_FORCE_INLINE void physecs::Constraint1DContainer::forEachConstraint(F&& f) {
    forEachList([&f](auto& constraintsList) PHYSECS_FORCE_INLINE_LAMBDA {
        for (auto& constraint : constraintsList.constraints) {
            f(constraint);
        }
    });
}

void physecs::Constraint1DContainer::preSolve(VelocityData* velocities, PseudoVelocityData* pseudoVelocities, float timeStep) {
    forEachConstraint([=](auto& constraint) PHYSECS_FORCE_INLINE_LAMBDA {
        constraint.preSolve(velocities, pseudoVelocities, timeStep);
    });
}

void physecs::Constraint1DContainer::solve(VelocityData* velocities, float baumgarteFactor) {
    forEachConstraint([=](auto& constraint) PHYSECS_FORCE_INLINE_LAMBDA {
        constraint.solve(velocities, baumgarteFactor);
    });
}

void physecs::Constraint1DContainer::clear() {
    forEachList([](auto& constraintsList) PHYSECS_FORCE_INLINE_LAMBDA {
        constraintsList.constraints.clear();
        constraintsList.lanes = _mm_setzero_si128();
    });
    constraintRefs.clear();
}
