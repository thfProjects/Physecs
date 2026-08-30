#pragma once

#include "PhysecsAPI.h"
#include "Components.h"

namespace physecs {
    PHYSECS_API glm::mat3 getInertiaSphere(float mass, float radius);
    PHYSECS_API glm::mat3 getInertiaCapsule(float mass, float halfHeight, float radius);
    PHYSECS_API glm::mat3 getInertiaBox(float mass, glm::vec3 halfExtents);
    PHYSECS_API glm::mat3 getInertiaTetrahedron(float mass, const std::array<glm::vec3, 4> &v);
    PHYSECS_API MassProps computeMassProps(const RigidBodyCollisionComponent& collisionComponent, float mass);
    PHYSECS_API void setMassProps(RigidBodyDynamicComponent& dynamicComponent, const RigidBodyCollisionComponent& collisionComponent, float mass);
}
