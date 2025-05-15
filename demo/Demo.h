#pragma once

#include <Camera.h>
#include <CameraMovementSystem.h>
#include <Light.h>
#include <entt.hpp>
#include <Physecs/Physecs.h>

class Demo {
    Camera camera;
    Light light;
    entt::registry registry;
    physecs::Scene physicsScene = physecs::Scene(registry);

    CameraMovementSystem cameraMovementSystem = CameraMovementSystem(camera);

    physecs::Material defaultMaterial = { 0.4f, 0.4f };

    const float physicsTimeStep = 1.0f / 240.0f;
    float physicsAccum = 0.f;

    entt::entity spawnBox(glm::vec3 position);
    entt::entity spawnCapsule(glm::vec3 position);
public:
    void onCreate();
    void onUpdate(float deltaTime);
    void draw();
};
