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
public:
    void onCreate();
    void onUpdate(float deltaTime);
    void draw();
};
