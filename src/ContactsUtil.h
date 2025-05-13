#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "ContactManifold.h"

namespace physecs {
    void generateContactsPolygonBoxFace(
        glm::vec3 boxCenter,
        glm::mat3 boxBasis,
        int boxAxis,
        int boxAxisSign,
        glm::vec3 boxHalfExtents,
        glm::vec3 incPlaneOrig,
        glm::vec3 incPlaneNormal,
        std::vector<glm::vec2>& polygon,
        int clipX,
        int clipY,
        ContactPoint* contactPoints,
        int& numPoints);

    void generateContactsPolygonPolygonFace(
        glm::vec3 refPos,
        glm::mat3& refToWorld,
        glm::vec3 refPlaneOrigin,
        glm::vec3 refPlaneNormal,
        glm::vec3 incPlaneOrigin,
        glm::vec3 incPlaneNormal,
        std::vector<glm::vec2>& polygon,
        int clipX,
        int clipY,
        ContactPoint* contactPoints,
        int& numPoints);
}
