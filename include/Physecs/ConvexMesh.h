#pragma once
#include <PhysecsAPI.h>
#include <vector>
#include <glm/glm.hpp>

namespace physecs {
    struct ConvexMeshFace {
        std::vector<int> indices;
        glm::vec3 normal;
        glm::vec3 centroid;
    };

    template<int N>
    class ConvexMeshVertices {
        std::vector<glm::vec3> buffer;
        int count;

        static int roundUp(int i) { return (i + N - 1) / N * N; }

    public:
        PHYSECS_API ConvexMeshVertices(const std::vector<glm::vec3>&& vertices);

        const glm::vec3& operator[](int index) const { return buffer[index]; }
        int size() const { return count; }
        std::vector<glm::vec3>::const_iterator begin() { return buffer.begin(); }
        std::vector<glm::vec3>::const_iterator end() { return buffer.end(); }
    };

    struct ConvexMesh {
        ConvexMeshVertices<4> vertices;
        std::vector<ConvexMeshFace> faces;

        ConvexMesh(const std::vector<glm::vec3>&& vertices, const std::vector<ConvexMeshFace>&& faces) : vertices(std::move(vertices)), faces(faces) {};
    };
}
