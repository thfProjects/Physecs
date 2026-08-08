#include "Constraint1DContainer.h"
#include "Constraint1D.cpp"
#include "Constraint1DW.cpp"
#include "SIMD.h"

namespace {
    template<typename Collection, typename F>
    void forEachList(Collection& collection, F&& f) {
        std::visit([&f](auto& constraintsCollection) {
            std::apply([&f](auto&... constraintsLists) {
                (f(constraintsLists), ...);
            }, constraintsCollection.constraints);
        }, collection);
    }

    template<typename Collection, typename F>
    void forEachConstraint(Collection& collection, F&& f) {
        forEachList(collection, [&f](auto& constraintsList) {
            for (auto& constraint : constraintsList.constraints) {
                f(constraint);
            }
        });
    }
}

void physecs::Constraint1DContainer::preSolve(const MassData* masses, VelocityData* velocities, PseudoVelocityData* pseudoVelocities) {
    forEachConstraint(constraintCollection, [=](auto& constraint) {
        constraint.preSolve(masses, velocities, pseudoVelocities);
    });
}

void physecs::Constraint1DContainer::solve(VelocityData* velocities, float timeStep, bool useBias) {
    forEachConstraint(constraintCollection, [=](auto& constraint) {
        constraint.solve(velocities, timeStep, useBias);
    });
}

void physecs::Constraint1DContainer::clear() {
    forEachList(constraintCollection, [](auto& constraintsList) {
        constraintsList.constraints.clear();
        constraintsList.lanes = _mm_setzero_si128();
    });
    constraintRefs.clear();
}
