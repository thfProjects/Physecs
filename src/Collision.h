#pragma once

#include <glm/glm.hpp>
#include "Colliders.h"
#include "ContactManifold.h"

namespace physecs {
    bool collision(glm::vec3 pos0, glm::quat or0, Geometry &geom0, glm::vec3 pos1, glm::quat or1, Geometry &geom1, std::vector<ContactManifold> &results);
}
