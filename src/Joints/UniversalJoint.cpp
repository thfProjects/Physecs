#include "UniversalJoint.h"
#include "JointsUtil.h"
#include "MathUtil.h"
#include "Joint.inl"

void physecs::UniversalJoint::makeConstraints(const JointWorldSpaceData &worldSpaceData, void* /*additionalData*/, Constraint1DDescriptor* constraintRows) {
    auto& [p0, p1, r0, r1, q0, q1] = worldSpaceData;

    const glm::vec3 u02 = col2(q0);
    const glm::vec3 u12 = col2(q1);
    const float d22 = glm::dot(u02, u12);
    const glm::vec3 u12xu02 = glm::cross(u12, u02);

    constraintRows[0].angular0 = u12xu02;
    constraintRows[0].angular1 = u12xu02;
    constraintRows[0].geometricError = d22;

    createPointToPointConstraint(p0, p1, r0, r1, constraintRows + 1);
}

namespace physecs {
    PHYSECS_DEFINE_JOINT_IMPL(UniversalJoint);
}
