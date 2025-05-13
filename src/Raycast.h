#pragma once

#include "Colliders.h"
#include "glm/glm.hpp"

bool intersectRayAABB(glm::vec3 rayOrig, glm::vec3 rayDir, glm::vec3 boxMin, glm::vec3 boxMax, float& t);
bool intersectRayGeometry(glm::vec3 rayOrig, glm::vec3 rayDir, glm::vec3 pos, glm::quat ori, const physecs::Geometry& geometry, float& t);
