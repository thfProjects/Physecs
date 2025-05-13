#include "Joint.h"
#include "Components.h"
#include "Transform.h"

void physecs::Joint::getJointData(entt::registry& registry, glm::vec3& p0, glm::vec3& p1, glm::vec3 &r0, glm::vec3 &r1, glm::mat3 &u0, glm::mat3 &u1) {
    auto dynamic0 = registry.try_get<RigidBodyDynamicComponent>(entity0);
    auto dynamic1 = registry.try_get<RigidBodyDynamicComponent>(entity1);

    auto& transform0 = registry.get<TransformComponent>(entity0);
    auto& transform1 = registry.get<TransformComponent>(entity1);

    glm::mat3 rot0 = glm::toMat3(transform0.orientation);
    glm::mat3 rot1 = glm::toMat3(transform1.orientation);

    glm::vec3 com0(0);
    if (dynamic0 && !dynamic0->isKinematic) {
        com0 = dynamic0->com;
    }

    glm::vec3 com1(0);
    if (dynamic1 && !dynamic1->isKinematic) {
        com1 = dynamic1->com;
    }

    u0 = rot0 * glm::toMat3(anchor0Or);
    u1 = rot1 * glm::toMat3(anchor1Or);

    r0 = rot0 * (anchor0Pos - com0);
    r1 = rot1 * (anchor1Pos - com1);

    p0 = transform0.position + rot0 * anchor0Pos;
    p1 = transform1.position + rot1 * anchor1Pos;
}
