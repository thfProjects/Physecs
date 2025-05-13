#pragma once

#include <vector>
#include <glm/glm.hpp>
#include "Bounds.h"

namespace physecs {

    struct Triangle {
        unsigned int indices[3];
        Bounds bounds;
        glm::vec3 normal;
        glm::vec3 centroid;
    };

    struct TriangleMeshBVHNode {
        Bounds bounds;
        int triCount = 0;
        int index;
    };

    class PHYSECS_API TriangleMesh {
        const static int numBuckets = 6;

        void updateNodeBounds(int nodeId);
        bool fillBuckets(int nodeId, int axis, Bounds* bucketsBounds, int* bucketsCounts);
        static bool evaluateSplit(int splitIndex, Bounds* bucketsBounds, int* bucketsCounts, float& cost, Bounds& boundsLeft, Bounds& boundsRight, int& countLeft, int& countRight);
        bool chooseSplit(int nodeId, int& bestAxis, int& splitIndex, Bounds& bestBoundsLeft, Bounds& bestBoundsRight, int& bestCountLeft, int& bestCountRight);
        void subdivide(int nodeId);
        void overlapBvhNode(const Bounds &bounds, int nodeId, std::vector<int>& overlapTriangles);
    public:
        std::vector<glm::vec3> vertices;
        std::vector<Triangle> triangles;
        std::vector<TriangleMeshBVHNode> bvh;

        const int rootId = 0;

        TriangleMesh(const std::vector<glm::vec3> &vertices, const std::vector<unsigned int>& indices);

        std::vector<int> overlapBvh(const Bounds &bounds);
    };
}
