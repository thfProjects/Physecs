#include "FixedJoint.h"
#include "Joint.inl"
#include "JointsUtil.h"

void physecs::FixedJoint::makeConstraints(const JointWorldSpaceData& worldSpaceData, void* /*additionalData*/, Constraint1DDescriptor* constraintRows) {
    auto& [p0, p1, r0, r1, q0, q1] = worldSpaceData;

    createAngularLockConstraint(q0, q1, constraintRows);
    createPointToPointConstraint(p0, p1, r0, r1, constraintRows + 3);
}

namespace physecs {
    PHYSECS_DEFINE_JOINT_IMPL(FixedJoint);
}
