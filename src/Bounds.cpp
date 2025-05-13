#include "Bounds.h"
#include <glm/gtx/quaternion.hpp>

glm::vec3 physecs::Bounds::getCenter() const {
    return (min + max) / 2.f;
}

glm::vec3 physecs::Bounds::getHalfExtents() const {
    return max - getCenter();
}

void physecs::Bounds::expand(glm::vec3 expansion) {
    if (expansion.x < 0) min.x += expansion.x;
    else max.x += expansion.x;

    if (expansion.y < 0) min.y += expansion.y;
    else max.y += expansion.y;

    if (expansion.z < 0) min.z += expansion.z;
    else max.z += expansion.z;
}

void physecs::Bounds::addMargin(glm::vec3 margin) {
    max += margin;
    min -= margin;
}

float physecs::Bounds::area() const {
    glm::vec3 d = max - min;
    return 2 * (d.x * d.y + d.y * d.z + d.z * d.x);
}
