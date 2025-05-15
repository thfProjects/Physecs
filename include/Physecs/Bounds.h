#pragma once

#include "PhysecsAPI.h"
#include <glm/glm.hpp>

namespace physecs {
    struct PHYSECS_API Bounds {
        glm::vec3 min = glm::vec3(0);
        glm::vec3 max = glm::vec3(0);

        glm::vec3 getCenter() const;
        glm::vec3 getHalfExtents() const;
        void expand(glm::vec3 expansion);
        void addMargin(glm::vec3 margin);
        float area() const;
    };
}
