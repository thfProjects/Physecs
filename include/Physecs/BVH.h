#pragma once

#include "Bounds.h"
#include <entt.hpp>

namespace physecs {

    struct BVHNode;

    class PHYSECS_API BVH {
    public:
        const static int null = -1;
    private:

        std::vector<int> idPool;
        std::vector<BVHNode> nodes;
        int rootId = null;

        int create();
        void branchAndBound(int nodeId, const Bounds& bounds, float inheritedCost, int& sBest, float& cBest, Bounds& newParentBounds);
        void rotate(int nodeId);
        void refit(int nodeId);
    public:
        int insert(entt::entity entity, int colliderIndex, const Bounds &bounds);
        void update(int nodeId, const Bounds& bounds);
        void remove(int nodeId);
        const std::vector<BVHNode>& getNodes() const;
        int getRootId() const;
    };

    struct InternalNodeData {
        int left = BVH::null;
        int right = BVH::null;
    };

    struct LeafNodeData {
        entt::entity entity = entt::null;
        int colliderIndex = 0;
    };

    struct BVHNode {
        Bounds bounds;
        int parent = BVH::null;
        bool isLeaf = false;
        union {
            InternalNodeData internal;
            LeafNodeData leaf;
        };
    };
}
