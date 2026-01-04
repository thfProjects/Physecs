#include "Joint.h"
#include "Components.h"
#include "Transform.h"

void physecs::JointSolverData::calculateWorldSpaceData(JointWorldSpaceData& data) const {
    data.u0 = glm::toMat3(transform0.orientation * anchor0Or);
    data.u1 = glm::toMat3(transform1.orientation * anchor1Or);

    data.r0 = transform0.orientation * r0;
    data.r1 = transform1.orientation * r1;

    data.p0 = transform0.position + transform0.orientation * anchor0Pos;
    data.p1 = transform1.position + transform1.orientation * anchor1Pos;
}
