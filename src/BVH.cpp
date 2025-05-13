#include "BVH.h"
#include "BoundsUtil.h"

int physecs::BVH::create() {
    if (idPool.empty()) {
        nodes.push_back(BVHNode{});
        return nodes.size() - 1;
    }

    int id = idPool.back();
    nodes[id] = BVHNode{};
    idPool.pop_back();
    return id;
}

void physecs::BVH::branchAndBound(int nodeId, const Bounds& bounds, float inheritedCost, int& sBest, float& cBest, Bounds& newParentBounds) {
    auto& node = nodes[nodeId];
    auto newNodeBounds = getUnion(node.bounds, bounds);
    float directCost = newNodeBounds.area();

    float cost = directCost + inheritedCost;

    if (cost < cBest) {
        sBest = nodeId;
        cBest = cost;
        newParentBounds = newNodeBounds;
    }

    if (node.isLeaf) return;

    inheritedCost += directCost - node.bounds.area();

    float cLow = bounds.area() + inheritedCost;

    if (cLow < cBest) {
        branchAndBound(node.internal.left, bounds, inheritedCost, sBest, cBest, newParentBounds);
        branchAndBound(node.internal.right, bounds, inheritedCost, sBest, cBest, newParentBounds);
    }
}

void physecs::BVH::rotate(int nodeId) {
    auto& node = nodes[nodeId];

    if (node.parent == null) return;

    auto& parent = nodes[node.parent];

    int siblingId = parent.internal.left == nodeId ? parent.internal.right : parent.internal.left;
    auto& sibling = nodes[siblingId];

    int child1Id = node.internal.left;
    int child2Id = node.internal.right;
    auto& child1 = nodes[child1Id];
    auto& child2 = nodes[child2Id];

    float sa = node.bounds.area();
    float saR1 = getUnion(child2.bounds, sibling.bounds).area();
    float saR2 = getUnion(child1.bounds, sibling.bounds).area();

    if (saR1 < saR2) {
        if (saR1 < sa) {
            //rotate with child1
            node.internal.left = siblingId;
            sibling.parent = nodeId;
            child1.parent = node.parent;
            if (siblingId == parent.internal.left) {
                parent.internal.left = child1Id;
            }
            else {
                parent.internal.right = child1Id;
            }

            node.bounds = getUnion(sibling.bounds, child2.bounds);
        }
    }
    else if (saR2 < sa) {
        //rotate with child2
        node.internal.right = siblingId;
        sibling.parent = nodeId;
        child2.parent = node.parent;
        if (siblingId == parent.internal.left) {
            parent.internal.left = child2Id;
        }
        else {
            parent.internal.right = child2Id;
        }

        node.bounds = getUnion(child1.bounds, sibling.bounds);
    }
}

void physecs::BVH::refit(int nodeId) {
    for (int refitId = nodeId; refitId != null; refitId = nodes[refitId].parent) {
        auto& refitNode = nodes[refitId];
        refitNode.bounds = getUnion(nodes[refitNode.internal.left].bounds, nodes[refitNode.internal.right].bounds);
        rotate(refitId);
    }
}

int physecs::BVH::insert(entt::entity entity, int colliderIndex, const Bounds& bounds) {
    int nodeId = create();

    nodes[nodeId].bounds = bounds;
    nodes[nodeId].isLeaf = true;
    nodes[nodeId].leaf.entity = entity;
    nodes[nodeId].leaf.colliderIndex = colliderIndex;

    if (rootId == null) {
        rootId = nodeId;
        return rootId;
    }

    auto& root = nodes[rootId];

    //find best sibling
    int bestSiblingId = rootId;
    auto newParentBounds = getUnion(bounds, root.bounds);
    float bestCost = newParentBounds.area();
    float inheritedCost = bestCost - root.bounds.area();

    branchAndBound(root.internal.left, bounds, inheritedCost, bestSiblingId, bestCost, newParentBounds);
    branchAndBound(root.internal.right, bounds, inheritedCost, bestSiblingId, bestCost, newParentBounds);

    //create new parent
    int oldParentId = nodes[bestSiblingId].parent;
    int newParentId = create();

    auto& newParent = nodes[newParentId];
    newParent.parent = oldParentId;
    newParent.bounds = newParentBounds;
    newParent.internal.left = bestSiblingId;
    newParent.internal.right = nodeId;
    nodes[bestSiblingId].parent = newParentId;
    nodes[nodeId].parent = newParentId;

    if (oldParentId != null) {
        auto& oldParent = nodes[oldParentId];
        if (oldParent.internal.left == bestSiblingId)
            oldParent.internal.left = newParentId;
        else
            oldParent.internal.right = newParentId;
    }
    else {
        rootId = newParentId;
    }

    //refit ancestors
    refit(newParentId);

    return nodeId;
}

void physecs::BVH::update(int nodeId, const Bounds &bounds) {
    auto& node = nodes[nodeId];
    node.bounds = bounds;
    refit(node.parent);
}

void physecs::BVH::remove(int nodeId) {

    if (nodeId == rootId) {
        rootId = null;
    }
    else {
        auto& node = nodes[nodeId];
        int parentId = node.parent;
        auto& parent = nodes[parentId];
        int siblingId = parent.internal.left == nodeId ? parent.internal.right : parent.internal.left;
        if (parentId == rootId) {
            rootId = siblingId;
            nodes[siblingId].parent = null;
        }
        else {
            int grandParentId = parent.parent;
            auto& grandParent = nodes[grandParentId];
            if (grandParent.internal.left == parentId) {
                grandParent.internal.left = siblingId;
            }
            else {
                grandParent.internal.right = siblingId;
            }
            nodes[siblingId].parent = grandParentId;
        }
        idPool.push_back(parentId);
    }
    idPool.push_back(nodeId);
}

const std::vector<physecs::BVHNode> & physecs::BVH::getNodes() const {
    return nodes;
}

int physecs::BVH::getRootId() const {
    return rootId;
}
