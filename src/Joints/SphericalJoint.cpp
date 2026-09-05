#include "SphericalJoint.h"
#include <JointsUtil.h>
#include "Joint.inl"

void physecs::SphericalJoint::makeConstraints(const JointWorldSpaceData &worldSpaceData, void* /*additionalData*/, Constraint1DDescriptor* constraintRows) {
    auto& [p0, p1, r0, r1, u0, u1] = worldSpaceData;

    createPointToPointConstraint(p0, p1, r0, r1, constraintRows);
}

namespace physecs {
    PHYSECS_DEFINE_JOINT_IMPL(SphericalJoint);
}
