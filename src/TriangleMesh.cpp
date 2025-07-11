#include "TriangleMesh.h"

#include <chrono>

#include "BoundsUtil.h"

void physecs::TriangleMesh::updateNodeBounds(int nodeId) {
    Bounds bounds{ glm::vec3(FLT_MAX), glm::vec3(-FLT_MAX) };
    auto& node = bvh[nodeId];
    for (int i = 0; i < node.triCount; ++i) {
        bounds = getUnion(bounds, triangles[node.index + i].bounds);
    }
    node.bounds = bounds;
}

bool physecs::TriangleMesh::fillBuckets(int nodeId, int axis, Bounds *bucketsBounds, int *bucketsCounts) {
    auto& node = bvh[nodeId];

    for (int i = 0; i < numBuckets; ++i) {
        bucketsBounds[i] = { glm::vec3(FLT_MAX), glm::vec3(-FLT_MAX) };
        bucketsCounts[i] = 0;
    }

    float boundsStart = node.bounds.min[axis];
    float boundsEnd = node.bounds.max[axis];

    float boundsLength = boundsEnd - boundsStart;

    if (!boundsLength) return false;

    for (int i = 0; i < node.triCount; ++i) {
        auto& tri = triangles[node.index + i];
        int bucketIndex = glm::min(numBuckets - 1, static_cast<int>((tri.centroid[axis] - boundsStart) / boundsLength * numBuckets));
        bucketsBounds[bucketIndex] = getUnion(bucketsBounds[bucketIndex], tri.bounds);
        bucketsCounts[bucketIndex]++;
    }

    return true;
}

bool physecs::TriangleMesh::evaluateSplit(int splitIndex, Bounds* bucketsBounds, int* bucketsCounts, float& cost, Bounds& boundsLeft, Bounds& boundsRight, int& countLeft, int& countRight) {
    boundsLeft = { glm::vec3(FLT_MAX), glm::vec3(-FLT_MAX) };
    boundsRight = { glm::vec3(FLT_MAX), glm::vec3(-FLT_MAX) };

    countLeft = 0;
    countRight = 0;

    for (int i = 0; i < numBuckets; ++i) {
        if (i < splitIndex) {
            boundsLeft = getUnion(boundsLeft, bucketsBounds[i]);
            countLeft += bucketsCounts[i];
        }
        else {
            boundsRight = getUnion(boundsRight, bucketsBounds[i]);
            countRight += bucketsCounts[i];
        }
    }

    if (!countLeft || !countRight) return false;

    cost = countLeft * boundsLeft.area() + countRight * boundsRight.area();

    return true;
}

bool physecs::TriangleMesh::chooseSplit(int nodeId, int& bestAxis, int& splitIndex, Bounds &bestBoundsLeft, Bounds &bestBoundsRight, int &bestCountLeft, int &bestCountRight) {
    float bestCost = FLT_MAX;

    for (int axis = 0; axis < 3; ++axis) {

        Bounds bucketsBounds [numBuckets];
        int bucketsCounts [numBuckets];

        if (!fillBuckets(nodeId, axis, bucketsBounds, bucketsCounts)) continue;;

        for (int i = 1; i < numBuckets; ++i) {
            float cost;
            Bounds boundsLeft;
            Bounds boundsRight;
            int countLeft = 0;
            int countRight = 0;
            if (evaluateSplit(i, bucketsBounds, bucketsCounts, cost, boundsLeft, boundsRight, countLeft, countRight)) {
                if (cost < bestCost) {
                    bestCost = cost;
                    bestAxis = axis;
                    splitIndex = i;
                    bestBoundsLeft = boundsLeft;
                    bestBoundsRight = boundsRight;
                    bestCountLeft = countLeft;
                    bestCountRight = countRight;
                }
            }
        }
    }

    return bestCountLeft && bestCountRight;
}

void physecs::TriangleMesh::subdivide(int nodeId) {
    auto& node = bvh[nodeId];

    int bestAxis = 0;
    int splitIndex = 0;

    Bounds bestBoundsLeft;
    Bounds bestBoundsRight;

    int bestCountLeft = 0;
    int bestCountRight = 0;

    //find best split index and axis
    if (!chooseSplit(nodeId, bestAxis, splitIndex, bestBoundsLeft, bestBoundsRight, bestCountLeft, bestCountRight)) return;

    //rearrange triangles around split
    float boundsStart = node.bounds.min[bestAxis];
    float boundsEnd = node.bounds.max[bestAxis];

    float boundsLength = boundsEnd - boundsStart;

    int i = node.index;
    int j = i + node.triCount - 1;
    while (i <= j) {
        auto& tri = triangles[i];
        int bucketIndex = static_cast<int>((tri.centroid[bestAxis] - boundsStart) / boundsLength * numBuckets);
        if (bucketIndex < splitIndex) {
            ++i;
        }
        else {
            std::swap(triangles[i], triangles[j--]);
        }
    }

    //create child nodes
    bvh.push_back({ bestBoundsLeft, bestCountLeft, node.index });
    bvh.push_back({ bestBoundsRight, bestCountRight, i });

    node.triCount = 0;
    node.index = bvh.size() - 2;

    subdivide(node.index);
    subdivide(node.index + 1);
}

physecs::TriangleMesh::TriangleMesh(const std::vector<glm::vec3> &vertices, const std::vector<unsigned int>& indices) {
    this->vertices = vertices;
    triangles.reserve(indices.size() / 3);
    for (unsigned int i = 0; i < indices.size(); i += 3) {
        unsigned int i0 = indices[i];
        unsigned int i1 = indices[i + 1];
        unsigned int i2 = indices[i + 2];
        auto& a = vertices[i0];
        auto& b = vertices[i1];
        auto& c = vertices[i2];
        Bounds bounds = getBoundsTriangle(a, b, c);
        glm::vec3 normal = glm::normalize(glm::cross(b - a, c - a));
        glm::vec3 centroid = (a + b + c) / 3.f;
        triangles.push_back({ i0, i1, i2, bounds, normal, centroid });
    }

    bvh.reserve(2.f * triangles.size() - 1);
    bvh.push_back({ {}, static_cast<int>(triangles.size()), 0 });
    updateNodeBounds(rootId);
    subdivide(rootId);
    stack.reserve(triangles.size());
}

const std::vector<int>& physecs::TriangleMesh::overlapBvh(const Bounds &bounds) {
    overlapTriangles.clear();

    stack.clear();
    stack.push_back(rootId);
    while (!stack.empty()) {
        const auto& node = bvh[stack.back()];
        stack.pop_back();
        if (intersects(bounds, node.bounds)) {
            if (node.triCount) {
                for (int i = 0; i < node.triCount; ++i) {
                    overlapTriangles.push_back(node.index + i);
                }
            }
            else {
                stack.push_back(node.index + 1);
                stack.push_back(node.index);
            }
        }
    }

    return overlapTriangles;
}
