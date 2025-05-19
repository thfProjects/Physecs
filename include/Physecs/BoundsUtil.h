#pragma once

#include "PhysecsAPI.h"
#include "Bounds.h"
#include "Colliders.h"
#include <entt.hpp>

namespace physecs {
    PHYSECS_API Bounds getBounds(entt::registry& registry, entt::entity entity);
    PHYSECS_API Bounds getBounds(glm::vec3 pos, glm::quat ori, const Geometry& geom);
    PHYSECS_API Bounds getBoundsSphere(glm::vec3 pos, float radius);
    PHYSECS_API Bounds getBoundsCapsule(glm::vec3 pos, glm::quat ori, float halfHeight, float radius);
    PHYSECS_API Bounds getBoundsBox(glm::vec3 pos, glm::quat ori, glm::vec3 halfExtents);
    PHYSECS_API Bounds getBoundsConvexMesh(glm::vec3 pos, glm::quat ori, ConvexMesh* mesh, glm::vec3 scale);
    PHYSECS_API Bounds getBoundsTriangle(glm::vec3 a, glm::vec3 b, glm::vec3 c);
    PHYSECS_API Bounds getBoundsTriangleMesh(glm::vec3 pos, glm::quat ori, TriangleMesh* mesh);
    PHYSECS_API bool intersects(const Bounds& a, const Bounds& b);
    PHYSECS_API Bounds getUnion(const Bounds& a, const Bounds& b);
}
