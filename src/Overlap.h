#pragma once

#include <glm/glm.hpp>

#include "Colliders.h"

namespace physecs {
    bool overlap(glm::vec3 pos0, glm::quat or0, Geometry& geom0, glm::vec3 pos1, glm::quat or1, Geometry& geom1);
}

