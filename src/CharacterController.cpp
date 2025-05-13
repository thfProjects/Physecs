#include "CharacterController.h"
#include "Physecs.h"
#include "Transform.h"

physecs::CharacterCollisionFlags physecs::CharacterController::move(glm::vec3 translation, float deltaTime) {
    auto& registry = scene.getRegistry();
    auto& transform = registry.get<TransformComponent>(entity);
    glm::vec3 up = transform.orientation * glm::vec3(0, 1, 0);
    glm::vec3 tVertical = up * glm::dot(up, translation);
    glm::vec3 tHorizontal = translation - tVertical;
    float thLength = glm::length(tHorizontal);
    if (thLength) {
        glm::vec3 dir = tHorizontal / thLength;

        entt::entity eBottom = scene.raycastClosest(transform.position + up * 0.01f, dir, radius + 0.1f);
        if (eBottom != entt::null) {

            entt::entity eTop = scene.raycastClosest(transform.position + up * maxStepHeight, dir, radius + 0.1f);
            if (eTop == entt::null) {
                translation = tHorizontal + up * deltaTime;
            }
        }
    }
    transform.position += translation;
    CharacterCollisionFlags result;
    Geometry geom { CAPSULE };
    geom.capsule = { height * 0.5f, radius };
    for (int i = 0; i < numIterations; ++i) {
        for (auto& [entity, colIndex, n, mtd] : scene.overlapWithMinTranslationalDistance(transform.position + up * (height * 0.5f + radius), transform.orientation, geom)) {
            float dot = glm::dot(n, up);
            if (dot > 0.001f) result.value |= CharacterCollisionFlags::DOWN;
            else if (dot < -0.001f) result.value |= CharacterCollisionFlags::UP;
            else result.value |= CharacterCollisionFlags::SIDES;

            if (dot > slopeLimit) {
                transform.position += up * mtd / dot;
            }
            else {
                transform.position += n * mtd;
            }
        }
    }
    registry.patch<TransformComponent>(entity);
    return result;
}
