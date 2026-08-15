#include "SphericalJoint.h"

#include <JointsUtil.h>

#include "Constraint1D.h"
#include "Constraint1DContainer.h"

void physecs::SphericalJoint::makeConstraints(const JointWorldSpaceData &worldSpaceData, void* /*additionalData*/, Constraint1DWriter& constraints) {
    auto& [p0, p1, r0, r1, u0, u1] = worldSpaceData;

    createPointToPointConstraint(p0, p1, r0, r1, constraints);
}

physecs::JointSolverDesc physecs::SphericalJoint::getSolverDesc(entt::registry &registry, Constraint1DLayout& constraintLayout) {
    constraintLayout.createConstraints<NONE, 3>(impulseCache.pointLambda);
    return {
        nullptr,
        makeConstraints
    };
}

void physecs::SphericalJoint::storeAccumulatedImpulses(Constraint1DReader& constraints) {
    for (float& lambda : impulseCache.pointLambda) lambda = constraints.nextTotalLambda<NONE>();
}
