#pragma once

#include <glm/glm.hpp>
#include "physecs.h"

namespace physecs {

    struct CharacterCollisionFlags {
        enum Enum { DOWN = 1, SIDES = 1 << 1, UP = 1 << 2 };
        char value = 0;
        bool isDown() { return value & DOWN; }
        bool isSide() { return value & SIDES; }
        bool isUp() { return value & UP; }
    };

    class PHYSECS_API CharacterController {
        Scene& scene;
        entt::entity entity = entt::null;
        float height = 0;
        float radius = 0;
        float maxStepHeight = 0.3f;
        float slopeLimit = glm::cos(glm::radians(45.f));
        int numIterations = 2;
    public:
        CharacterController(Scene& scene, entt::entity entity, float height, float radius, float maxStepHeight = 0.3f, float slopeLimit = glm::cos(glm::radians(45.f)), int numIterations = 2) : scene(scene), entity(entity), height(height), radius(radius), maxStepHeight(maxStepHeight), slopeLimit(slopeLimit), numIterations(numIterations) {}
        CharacterController(Scene& scene) : scene(scene){}
        void setEntity(entt::entity entity) { this->entity = entity; }
        void setHeight(float height) { this->height = height; }
        void setRadius(float radius) { this->radius = radius; }
        void setMaxStepHeight(float maxStepHeight) { this->maxStepHeight = maxStepHeight; }
        void setSlopeLimit(float slopeLimit) { this->slopeLimit = slopeLimit; }
        void setNumIterations(int numIterations) { this->numIterations = numIterations; }
        entt::entity getEntity() const { return entity; }
        float getHeight() const { return height; }
        float getRadius() const { return radius; }
        float getMaxStepHeight() const { return maxStepHeight; }
        float getSlopeLimit() const { return slopeLimit; }
        int getNumIterations() const { return numIterations; }
        CharacterCollisionFlags move(glm::vec3 translation, float deltaTime);
    };

}
