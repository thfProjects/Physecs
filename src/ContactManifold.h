#pragma once

#include <glm/glm.hpp>

namespace physecs {
    struct ContactPoint {
        glm::vec3 position0;
        glm::vec3 position1;
    };

    struct ContactManifold {
        int numPoints = 0;
        ContactPoint points[4];
        glm::vec3 normal;
        int triangleIndex = -1;
    };
}
