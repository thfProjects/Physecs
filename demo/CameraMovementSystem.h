#pragma once

#include "Camera.h"

class CameraMovementSystem {
    Camera& camera;
    bool rotateStarted = false;
    float rotateFactor = 0.08f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float movementSpeed = 2.0f;
public:
    CameraMovementSystem(Camera& camera) : camera(camera) {}
    void onUpdate(float deltaTime);
};
