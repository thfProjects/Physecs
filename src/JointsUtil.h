#pragma once

#include "Constraint1DContainer.h"

namespace physecs {

    // creates 3 1D constraints
    inline void createPointToPointConstraint(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& r0, const glm::vec3& r1, Constraint1DWriter& constraintWriter) {
        const glm::vec3 d = p1 - p0;

        constraintWriter.next()
        .setLinear(glm::vec3(1, 0, 0))
        .setAngular0(glm::vec3(0, r0.z, -r0.y))
        .setAngular1(glm::vec3(0, r1.z, -r1.y))
        .setC(d.x);

        constraintWriter.next()
        .setLinear(glm::vec3(0, 1, 0))
        .setAngular0(glm::vec3(-r0.z, 0, r0.x))
        .setAngular1(glm::vec3(-r1.z, 0, r1.x))
        .setC(d.y);

        constraintWriter.next()
        .setLinear(glm::vec3(0, 0, 1))
        .setAngular0(glm::vec3(r0.y, -r0.x, 0))
        .setAngular1(glm::vec3(r1.y, -r1.x, 0))
        .setC(d.z);
    }
}
