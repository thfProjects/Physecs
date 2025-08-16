#include "PrismaticJoint.h"
#include "Constraint1D.h"

void physecs::PrismaticJoint::setUpperLimit(float upperLimit) {
    this->upperLimit = upperLimit;
}

void physecs::PrismaticJoint::setLowerLimit(float lowerLimit) {
    this->lowerLimit = lowerLimit;
}

void physecs::PrismaticJoint::setDriveEnabled(bool driveEnabled) {
    this->driveEnabled = driveEnabled;
}

void physecs::PrismaticJoint::setTargetPosition(float targetPosition) {
    this->targetPosition = targetPosition;
}

void physecs::PrismaticJoint::setDriveStiffness(float driveStiffness) {
    this->driveStiffness = driveStiffness;
}

void physecs::PrismaticJoint::setDriveDamping(float driveDamping) {
    this->driveDamping = driveDamping;
}

void physecs::PrismaticJoint::makeConstraints(Constraint1D *constraints) {
    glm::vec3 p0, p1, r0, r1;
    glm::mat3 u0, u1;
    getJointData(p0, p1, r0, r1, u0, u1);

    glm::vec3 d = p1 - p0;

    //translation
    float dy = glm::dot(d, u0[1]);
    glm::vec3 r0xy = glm::cross(r0, u0[1]);
    glm::vec3 r1xy = glm::cross(r1, u0[1]);

    constraints[0].n = u0[1];
    constraints[0].r0xn = r0xy;
    constraints[0].r1xn = r1xy;
    constraints[0].c = dy;

    float dz = glm::dot(d, u0[2]);
    glm::vec3 r0xz = glm::cross(r0, u0[2]);
    glm::vec3 r1xz = glm::cross(r1, u0[2]);

    constraints[1].n = u0[2];
    constraints[1].r0xn = r0xz;
    constraints[1].r1xn = r1xz;
    constraints[1].c = dz;

    //rotation
    float d01 = glm::dot(u0[0], u1[1]);
    glm::vec3 u11xu00 = glm::cross(u1[1], u0[0]);

    constraints[2].r0xn = u11xu00;
    constraints[2].r1xn = u11xu00;
    constraints[2].c = d01;
    constraints[2].flags |= Constraint1D::ANGULAR;

    float d02 = glm::dot(u0[0], u1[2]);
    glm::vec3 u12xu00 = glm::cross(u1[2], u0[0]);

    constraints[3].r0xn = u12xu00;
    constraints[3].r1xn = u12xu00;
    constraints[3].c = d02;
    constraints[3].flags |= Constraint1D::ANGULAR;

    float d12 = glm::dot(u0[1], u1[2]);
    glm::vec3 u12xu01 = glm::cross(u1[2], u0[1]);

    constraints[4].r0xn = u12xu01;
    constraints[4].r1xn = u12xu01;
    constraints[4].c = d12;
    constraints[4].flags |= Constraint1D::ANGULAR;

    float dx = glm::dot(d, u0[0]);
    glm::vec3 r0xx = glm::cross(r0, u0[0]);
    glm::vec3 r1xx = glm::cross(r1, u0[0]);

    //limits
    int index = 5;
    if (makeUpperLimit) {
        constraints[index].n = u0[0];
        constraints[index].r0xn = r0xx;
        constraints[index].r1xn = r1xx;
        constraints[index].c = dx - upperLimit;
        constraints[index].min = 0;
        constraints[index].flags |= Constraint1D::LIMITED;

        index++;
    }
    else if (makeLowerLimit) {
        constraints[index].n = u0[0];
        constraints[index].r0xn = r0xx;
        constraints[index].r1xn = r1xx;
        constraints[index].c = dx - lowerLimit;
        constraints[index].max = 0;
        constraints[index].flags |= Constraint1D::LIMITED;

        index++;
    }

    //drive
    if (driveEnabled) {
        constraints[index].n = u0[0];
        constraints[index].r0xn = r0xx;
        constraints[index].r1xn = r1xx;
        constraints[index].c = dx - targetPosition;
        constraints[index].flags |= Constraint1D::SOFT;
        constraints[index].frequency = driveStiffness;
        constraints[index].dampingRatio = driveDamping;
    }
}

void physecs::PrismaticJoint::calculateNumConstraints(entt::registry& registry) {
    glm::vec3 p0, p1, r0, r1;
    glm::mat3 u0, u1;
    getJointData(p0, p1, r0, r1, u0, u1);

    glm::vec3 d = p1 - p0;

    float dx = glm::dot(d, u0[0]);

    if (dx > upperLimit) {
        makeUpperLimit = true;
        makeLowerLimit = false;
    }
    else if (dx < lowerLimit) {
        makeUpperLimit = false;
        makeLowerLimit = true;
    }
    else {
        makeUpperLimit = false;
        makeLowerLimit = false;
    }

    numConstraints = 5 + (makeUpperLimit || makeLowerLimit) + driveEnabled;
}
