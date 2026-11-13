#include "UniversalJoint.h"
#include "JointsUtil.h"
#include "Joint.inl"

// third column of rotation matrix of quat q
static glm::vec3 col2(const glm::quat& q) {
    return glm::vec3(
        2.f * (q.x * q.z + q.w * q.y),
        2.f * (q.y * q.z - q.w * q.x),
        1.f - 2.f * (q.x * q.x + q.y * q.y));
}

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
