#include "Clipping.h"

#include <glm/gtx/vector_angle.hpp>

namespace {
    const int MAX_POINTS = 128;
}

static bool isInside(glm::vec2 point, glm::vec2 a, glm::vec2 b) {
    return (b.y - a.y) * (point.x - a.x) - (b.x - a.x) * (point.y - a.y) > 0; //dot product with normal of edge towards interior
}

static glm::vec2 intersection(glm::vec2 a1, glm::vec2 b1, glm::vec2 a2, glm::vec2 b2) {
    auto r1 = b1 - a1;
    auto r2 = b2 - a2;
    float t = glm::determinant(glm::mat2(a1 - a2, r2)) / glm::determinant(glm::mat2(-r1, r2));
    return a1 + t * r1;
}

static void clipEdge(std::vector<glm::vec2>& polygon, glm::vec2 a, glm::vec2 b) {
    glm::vec2 newPoints[MAX_POINTS];
    int newPointCount = 0;

    for (int i = 0; i < polygon.size(); ++i) {
        int j = (i + 1) % polygon.size();

        bool iInside = isInside(polygon[i], a, b);
        bool jInside = isInside(polygon[j], a, b);

        if (iInside && jInside) {
            newPoints[newPointCount++] = polygon[j];
        } else if (iInside) {
            newPoints[newPointCount++] = intersection(polygon[i], polygon[j], a, b);
        } else if (jInside) {
            newPoints[newPointCount++] = intersection(polygon[i], polygon[j], a, b);
            newPoints[newPointCount++] = polygon[j];
        }
    }

    polygon.resize(newPointCount);

    for (int i = 0; i < newPointCount; ++i) {
        polygon[i] = newPoints[i];
    }
}

void suthHodgClip(std::vector<glm::vec2>& polygon, std::vector<glm::vec2>& clip) {
    for (int i = 0; i < clip.size(); ++i) {
        int j = (i + 1) % clip.size();
        clipEdge(polygon, clip[i], clip[j]);
    }
}

bool clipLine(glm::vec2& p0, glm::vec2& p1, glm::vec2* clip, int clipSize) {
    for (int i = 0; i < clipSize; ++i) {
        int j = (i + 1) % clipSize;

        bool isInside0 = isInside(p0, clip[i], clip[j]);
        bool isInside1 = isInside(p1, clip[i], clip[j]);

        if (isInside0 && isInside1) continue;

        if (isInside0)
            p1 = intersection(p0, p1, clip[i], clip[j]);
        else if (isInside1)
            p0 = intersection(p0, p1, clip[i], clip[j]);
        else return false;
    }

    return true;
}
