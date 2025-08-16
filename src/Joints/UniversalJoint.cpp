#include "UniversalJoint.h"
#include "Constraint1D.h"

void physecs::UniversalJoint::makeConstraints(Constraint1D* constraints) {
    glm::vec3 p0, p1, r0, r1;
    glm::mat3 u0, u1;
    getJointData(p0, p1, r0, r1, u0, u1);

    glm::vec3 d = p1 - p0;

    float cn = glm::dot(d, d);
    glm::vec3 r0xd = glm::cross(r0, d);
    glm::vec3 r1xd = glm::cross(r1, d);

    constraints[0].n = d;
    constraints[0].r0xn = r0xd;
    constraints[0].r1xn = r1xd;
    constraints[0].c = cn;

    float d22 = glm::dot(u0[2], u1[2]);
    glm::vec3 u12xu02 = glm::cross(u1[2], u0[2]);

    constraints[1].r0xn = u12xu02;
    constraints[1].r1xn = u12xu02;
    constraints[1].c = d22;
    constraints[1].flags |= Constraint1D::ANGULAR;
}
