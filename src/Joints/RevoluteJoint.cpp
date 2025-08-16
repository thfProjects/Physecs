#include "RevoluteJoint.h"
#include "Constraint1D.h"

void physecs::RevoluteJoint::setDriveEnabled(bool enabled) {
    driveEnabled = enabled;
}

void physecs::RevoluteJoint::setDriveVelocity(float velocity) {
    driveVelocity = velocity;
}

void physecs::RevoluteJoint::makeConstraints(Constraint1D* constraints) {
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

    float d01 = glm::dot(u0[0], u1[1]);
    glm::vec3 u11xu00 = glm::cross(u1[1], u0[0]);

    constraints[1].r0xn = u11xu00;
    constraints[1].r1xn = u11xu00;
    constraints[1].c = d01;
    constraints[1].flags |= Constraint1D::ANGULAR;

    float d02 = glm::dot(u0[0], u1[2]);
    glm::vec3 u12xu00 = glm::cross(u1[2], u0[0]);

    constraints[2].r0xn = u12xu00;
    constraints[2].r1xn = u12xu00;
    constraints[2].c = d02;
    constraints[2].flags |= Constraint1D::ANGULAR;

    if (driveEnabled) {
        constraints[3].r0xn = u0[0];
        constraints[3].r1xn = u0[0];
        constraints[3].targetVelocity = driveVelocity;
        constraints[3].flags |= Constraint1D::ANGULAR;
    }
}

void physecs::RevoluteJoint::calculateNumConstraints(entt::registry &registry) {
    numConstraints = driveEnabled ? 4 : 3;
}
