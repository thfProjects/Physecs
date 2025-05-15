#pragma once

#include <glm/glm.hpp>

struct Light {
	glm::vec3 direction;
	glm::vec3 color;

	Light(glm::vec3 direction, glm::vec3 color): direction(direction), color(color) {}
	Light() : direction(glm::vec3(0, -1, 0)), color(glm::vec3(1)) {}
};
