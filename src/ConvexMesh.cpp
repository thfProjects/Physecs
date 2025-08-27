#include "ConvexMesh.h"

template<int N>
physecs::ConvexMeshVertices<N>::ConvexMeshVertices(const std::vector<glm::vec3> && vertices) {
    count = vertices.size();
    buffer = std::move(vertices);

    const int newSize = roundUp(count);
    const glm::vec3 val = buffer.back();
    buffer.resize(newSize, val);
}
