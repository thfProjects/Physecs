#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

struct TransformComponent {
    glm::vec3 position;
    glm::quat orientation;
    glm::vec3 scale;
};
