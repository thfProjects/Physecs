#pragma once
#include <vector>
#include <glm/glm.hpp>

namespace physecs {
    struct ConvexMeshFace {
        std::vector<int> indices;
        glm::vec3 normal;
        glm::vec3 centroid;
    };

    struct ConvexMesh {
        std::vector<glm::vec3> vertices;
        std::vector<ConvexMeshFace> faces;

        //ConvexMesh(const std::vector<glm::vec3>& pointCloud);
        ConvexMesh(const std::vector<glm::vec3>& vertices, const std::vector<ConvexMeshFace>& faces) : vertices(vertices), faces(faces) {};
    };
}
