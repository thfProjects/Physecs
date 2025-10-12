#include "Joint.h"
#include "Components.h"
#include "Transform.h"

void physecs::JointSolverData::calculateWorldSpaceData(JointWorldSpaceData& data) const {
    const glm::mat3 rot0 = glm::toMat3(transform0.orientation);
    const glm::mat3 rot1 = glm::toMat3(transform1.orientation);

    data.u0 = rot0 * glm::toMat3(anchor0Or);
    data.u1 = rot1 * glm::toMat3(anchor1Or);

    data.r0 = rot0 * r0;
    data.r1 = rot1 * r1;

    data.p0 = transform0.position + rot0 * anchor0Pos;
    data.p1 = transform1.position + rot1 * anchor1Pos;
}
