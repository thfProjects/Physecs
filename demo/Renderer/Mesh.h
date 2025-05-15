#pragma once

#include <vector>
#include <glm/glm.hpp>

struct Vertex {
	glm::vec3 position;
	glm::vec3 normal;
	glm::vec2 texCoords;
};

struct ScreenVertex {
	glm::vec3 position;
	glm::vec2 texCoords;
};

unsigned int createMesh(const std::vector<Vertex> &vertices, const std::vector<unsigned int> &indices);
unsigned int createMesh(const std::vector<ScreenVertex> &vertices, const std::vector<unsigned int> &indices);

