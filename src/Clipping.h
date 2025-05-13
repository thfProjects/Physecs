#pragma once
#include <vector>
#include <glm/glm.hpp>

void suthHodgClip(std::vector<glm::vec2>& polygon, std::vector<glm::vec2>& clip);
bool clipLine(glm::vec2& p0, glm::vec2& p1, glm::vec2* clip, int clipSize);
