#include "SphericalJoint.h"
#include "Constraint1D.h"

void physecs::SphericalJoint::makeConstraints(Constraint1D* constraints, entt::registry &registry) {
    glm::vec3 p0, p1, r0, r1;
    glm::mat3 u0, u1;
    getJointData(registry, p0, p1, r0, r1, u0, u1);

    glm::vec3 d = p1 - p0;

    float cn = glm::dot(d, d);
    glm::vec3 r0xd = glm::cross(r0, d);
    glm::vec3 r1xd = glm::cross(r1, d);

    constraints[0].n = d;
    constraints[0].r0xn = r0xd;
    constraints[0].r1xn = r1xd;
    constraints[0].c = cn;
}
