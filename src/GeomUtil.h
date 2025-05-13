#pragma once

#include <utility>
#include <glm/glm.hpp>

enum class TriangleFeature : char {
    FACE, EDGE, VERTEX
};

glm::vec3 closestPointOnSegment(glm::vec3 point, glm::vec3 lineOrig, glm::vec3 lineDir, float min, float max);
std::pair<glm::vec3, glm::vec3> closestPointsBetweenSegments(glm::vec3 lineOrig0, glm::vec3 dir0, float min0, float max0, glm::vec3 lineOrig1, glm::vec3 dir1, float min1, float max1);
std::pair<glm::vec3, glm::vec3> closestPointsBetweenSegments(glm::vec3 lineOrig0, glm::vec3 v0, glm::vec3 lineOrig1, glm::vec3 v1);
float sqrDistSegmentAABB(glm::vec3 p, glm::vec3 dir, float min, float max, glm::vec3 halfExtents, float& t, glm::vec3& q);
float sqrDistPointTriangle(glm::vec3 p, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3& q, TriangleFeature &triangleFeature, char &featureIndex);
float sqrDistSegmentTriangle(glm::vec3 p, glm::vec3 dir, float min, float max, glm::vec3 a, glm::vec3 b, glm::vec3 c, float& t, glm::vec3& q, TriangleFeature& triangleFeature, char& featureIndex);
float distanceAABBPlane(glm::vec3 halfExtents, glm::vec3 n, float d);