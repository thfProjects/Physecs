#include "CameraMovementSystem.h"
#include "Input.h"

void CameraMovementSystem::onUpdate(float deltaTime) {
    glm::vec2 mouseDeltaPos = Input::getMouseDeltaPos();

    if (rotateStarted && (mouseDeltaPos.x || mouseDeltaPos.y)) {
        pitch += glm::radians(-mouseDeltaPos.y) * rotateFactor;
        yaw += glm::radians(mouseDeltaPos.x) * rotateFactor;
        camera.orientation = glm::quat(glm::vec3(pitch, yaw, 0.0f));
    }

    if (Input::getMouseDown(SDL_BUTTON_RIGHT)) {
        rotateStarted = true;
    }

    if (Input::getMouseUp(SDL_BUTTON_RIGHT)) {
        rotateStarted = false;
    }

    if (Input::getKey(SDL_SCANCODE_W)) {
        camera.position += camera.orientation * camera.defaultDir * movementSpeed * deltaTime;
    }

    if (Input::getKey(SDL_SCANCODE_S)) {
        camera.position -= camera.orientation * camera.defaultDir * movementSpeed * deltaTime;
    }

    if (Input::getKey(SDL_SCANCODE_D)) {
        camera.position += camera.orientation * glm::vec3(1, 0, 0) * movementSpeed * deltaTime;
    }

    if (Input::getKey(SDL_SCANCODE_A)) {
        camera.position -= camera.orientation * glm::vec3(1, 0, 0) * movementSpeed * deltaTime;
    }

    if (Input::getKey(SDL_SCANCODE_SPACE)) {
        camera.position += camera.orientation * glm::vec3(0, 1, 0) * movementSpeed * deltaTime;
    }

    if (Input::getKey(SDL_SCANCODE_LSHIFT)) {
        camera.position -= camera.orientation * glm::vec3(0, 1, 0) * movementSpeed * deltaTime;
    }
}
