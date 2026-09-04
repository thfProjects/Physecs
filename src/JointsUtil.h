#pragma once

#include "Joint.h"

namespace physecs {

    // fills 3 1D constraints
    inline void createPointToPointConstraint(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& r0, const glm::vec3& r1, Constraint1DDescriptor* constraintRows) {
        const glm::vec3 d = p1 - p0;

        // prevents creating net torque on body pair, forces must be applied on same point
        const glm::vec3 a0 = r0 + d;

        constraintRows[0].linear0 = glm::vec3(1, 0, 0);
        constraintRows[0].linear1 = glm::vec3(1, 0, 0);
        constraintRows[0].angular0 = glm::vec3(0, a0.z, -a0.y);
        constraintRows[0].angular1 = glm::vec3(0, r1.z, -r1.y);
        constraintRows[0].geometricError = d.x;

        constraintRows[1].linear0 = glm::vec3(0, 1, 0);
        constraintRows[1].linear1 = glm::vec3(0, 1, 0);
        constraintRows[1].angular0 = glm::vec3(-a0.z, 0, a0.x);
        constraintRows[1].angular1 = glm::vec3(-r1.z, 0, r1.x);
        constraintRows[1].geometricError = d.y;

        constraintRows[2].linear0 = glm::vec3(0, 0, 1);
        constraintRows[2].linear1 = glm::vec3(0, 0, 1);
        constraintRows[2].angular0 = glm::vec3(a0.y, -a0.x, 0);
        constraintRows[2].angular1 = glm::vec3(r1.y, -r1.x, 0);
        constraintRows[2].geometricError = d.z;
    }
}
