#pragma once

#include <glm/glm.hpp>
#include "Colliders.h"
#include "ContactManifold.h"

namespace physecs {
    bool collisionTriangleMesh(glm::vec3 pos0, glm::quat or0, const Geometry& geom0, glm::vec3 pos1, glm::quat or1, TriangleMesh* mesh1, std::vector<ContactManifold>& results);
}
