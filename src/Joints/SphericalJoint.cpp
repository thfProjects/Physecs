#include "SphericalJoint.h"

#include <JointsUtil.h>

#include "Constraint1D.h"
#include "Constraint1DContainer.h"

void physecs::SphericalJoint::makeConstraints(const JointWorldSpaceData &worldSpaceData, void* /*additionalData*/, Constraint1DWriter& constraints) {
    auto& [p0, p1, r0, r1, u0, u1] = worldSpaceData;

    createPointToPointConstraint(p0, p1, r0, r1, constraints);
}
