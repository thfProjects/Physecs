#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

struct Camera {
    glm::vec3 position;
    glm::quat orientation;
    float fov;
    static constexpr glm::vec3 defaultDir = glm::vec3(0, 0, -1);

    Camera(glm::vec3 position, glm::quat orientation, float fov) : position(position), orientation(orientation), fov(fov) {}
    Camera() : position(glm::vec3(0)), orientation(glm::quat(1, 0, 0, 0)), fov(45.0f) {}
};
