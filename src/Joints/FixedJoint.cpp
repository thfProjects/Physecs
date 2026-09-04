#include "FixedJoint.h"
#include "Constraint1D.h"
#include "Constraint1DContainer.h"
#include "JointsUtil.h"

void physecs::FixedJoint::makeConstraints(const JointWorldSpaceData& worldSpaceData, void* /*additionalData*/, Constraint1DDescriptor* constraintRows) {
    auto& [p0, p1, r0, r1, u0, u1] = worldSpaceData;

    const float d01 = glm::dot(u0[0], u1[1]);
    const glm::vec3 u11xu00 = glm::cross(u1[1], u0[0]);

    constraintRows[0].angular0 = u11xu00;
    constraintRows[0].angular1 = u11xu00;
    constraintRows[0].geometricError = d01;

    const float d02 = glm::dot(u0[0], u1[2]);
    const glm::vec3 u12xu00 = glm::cross(u1[2], u0[0]);

    constraintRows[1].angular0 = u12xu00;
    constraintRows[1].angular1 = u12xu00;
    constraintRows[1].geometricError = d02;

    const float d12 = glm::dot(u0[1], u1[2]);
    const glm::vec3 u12xu01 = glm::cross(u1[2], u0[1]);

    constraintRows[2].angular0 = u12xu01;
    constraintRows[2].angular1 = u12xu01;
    constraintRows[2].geometricError = d12;

    createPointToPointConstraint(p0, p1, r0, r1, constraintRows + 3);
}

namespace physecs {
    PHYSECS_DEFINE_JOINT_IMPL(FixedJoint);
}
