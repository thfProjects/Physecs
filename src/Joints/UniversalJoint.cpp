#include "UniversalJoint.h"

#include <JointsUtil.h>

#include "Constraint1D.h"
#include "Constraint1DContainer.h"

void physecs::UniversalJoint::makeConstraints(const JointWorldSpaceData &worldSpaceData, void* /*additionalData*/, Constraint1DDescriptor* constraintRows) {
    auto& [p0, p1, r0, r1, u0, u1] = worldSpaceData;

    const float d22 = glm::dot(u0[2], u1[2]);
    const glm::vec3 u12xu02 = glm::cross(u1[2], u0[2]);

    constraintRows[0].angular0 = u12xu02;
    constraintRows[0].angular1 = u12xu02;
    constraintRows[0].geometricError = d22;

    createPointToPointConstraint(p0, p1, r0, r1, constraintRows + 1);
}

namespace physecs {
    PHYSECS_DEFINE_JOINT_IMPL(UniversalJoint);
}
