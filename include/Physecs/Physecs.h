#pragma once

#include <entt.hpp>
#include <glm/glm.hpp>
#include "BVH.h"
#include "Joint.h"
#include "Colliders.h"

namespace physecs {
    struct ContactManifold;
    struct ContactConstraints;

    struct OverlapHit {
        entt::entity entity;
        int colIndex;
    };

    struct OverlapMtdHit {
        entt::entity entity;
        int colIndex;
        glm::vec3 normal;
        float mtd;
    };

    class OnTriggerEnterListener {
    public:
        virtual void onTriggerEnter(entt::entity, int, entt::entity, int) = 0;
        virtual ~OnTriggerEnterListener() = default;
    };

    class OnTriggerExitListener {
    public:
        virtual void onTriggerExit(entt::entity, int, entt::entity, int) = 0;
        virtual ~OnTriggerExitListener() = default;
    };

    enum ContactType { COLLISION, TRIGGER };
    PHYSECS_API ContactType defaultContactFilter (bool isTrigger0, int data0, bool isTrigger1, int data1);

    class Scene {

        struct EntityPair {
            entt::entity entity0;
            entt::entity entity1;

            bool operator==(const EntityPair& other) const {
                return entity0 == other.entity0 && entity1 == other.entity1;
            }
        };

        struct EntityPairHash {
            std::size_t operator()(const EntityPair& pair) const {
                return std::hash<uint64_t>{}(static_cast<uint64_t>(pair.entity0) << 32 | static_cast<uint64_t>(pair.entity1));
            }
        };

        struct BroadPhaseEntryKey {
            entt::entity entity;
            int colliderIndex;

            bool operator==(const BroadPhaseEntryKey& other) const {
                return entity == other.entity && colliderIndex == other.colliderIndex;
            }
        };

        struct BroadPhaseEntryHash {
            std::size_t operator()(const BroadPhaseEntryKey& key) const {
                std::size_t h1 = std::hash<int>{}(static_cast<int>(key.entity));
                std::size_t h2 = std::hash<int>{}(key.colliderIndex);

                return h1 ^ h2 << 1;
            }
        };

        struct BroadPhaseEntry {
            entt::entity entity;
            int colliderIndex;
            Bounds bounds;
            int nodeId;
            bool nodeDirty;
            bool enableSimulation;
        };

        struct ContactPair {
            entt::entity entity0;
            int colliderIndex0;
            entt::entity entity1;
            int colliderIndex1;

            bool operator==(const ContactPair& other) const {
                return entity0 == other.entity0 && colliderIndex0 == other.colliderIndex0 && entity1 == other.entity1 && colliderIndex1 == other.colliderIndex1;
            }
        };

        struct ContactHash {
            std::size_t operator()(const ContactPair& pair) const {
                std::size_t h1 = std::hash<int>{}(static_cast<int>(pair.entity0));
                std::size_t h2 = std::hash<int>{}(pair.colliderIndex0);
                std::size_t h3 = std::hash<int>{}(static_cast<int>(pair.entity0));
                std::size_t h4 = std::hash<int>{}(pair.colliderIndex1);

                return h1 ^ h2 << 1 ^ (h3 ^ h4 << 1);
            }
        };

        struct CollisionPair {
            ContactPair contactPair;
            int triangleIndex;

            bool operator==(const CollisionPair& other) const {
                return contactPair == other.contactPair && triangleIndex == other.triangleIndex;
            }
        };

        struct CollisionHash {
            std::size_t operator()(const CollisionPair& pair) const {
                return ContactHash{}(pair.contactPair) << 32 | pair.triangleIndex;
            }
        };

        struct ContactPointData {
            glm::vec3 localPosition0;
            float targetVelocity;
        };

        struct ContactManifoldData {
            int numPoints;
            ContactPointData contactPointData[4];
        };

        entt::registry& registry;

        int numSubSteps = 8;
        int numIterations = 2;

        float g = 9.81f;

        std::vector<BroadPhaseEntry> broadPhaseEntries;
        std::vector<ContactPair> potentialContacts;
        std::vector<ContactManifold> contactBuffer;
        std::vector<ContactConstraints> contactConstraints;
        std::vector<Constraint1D> jointConstraints;
        std::unordered_map<CollisionPair, ContactManifoldData, CollisionHash> contactCache;
        std::unordered_map<CollisionPair, ContactManifoldData, CollisionHash> contactCacheTemp;
        BVH bvh;
        std::unordered_map<BroadPhaseEntryKey, int, BroadPhaseEntryHash> colToBroadPhaseEntry;
        std::vector<Joint*> joints;
        std::unordered_set<EntityPair, EntityPairHash> nonCollidingPairs;
        std::unordered_set<ContactPair, ContactHash> triggerCache;
        std::unordered_set<ContactPair, ContactHash> triggerCacheTemp;
        std::vector<OnTriggerEnterListener*> onTriggerEnterCallbacks;
        std::vector<OnTriggerExitListener*> onTriggerExitCallbacks;
        ContactType (*contactFilter)(bool, int, bool, int) = defaultContactFilter;
        std::vector<glm::vec3> contactPoints;

        void onRigidBodyCreate(entt::registry& registry, entt::entity entity);
        void onRigidBodyDelete(entt::registry& registry, entt::entity entity);
        void onRigidBodyMove(entt::registry& registry, entt::entity entity);

        void updateBounds(entt::entity entity);
        void updateBVH();

        entt::entity raycastClosestBVHNode(glm::vec3 rayOrig, glm::vec3 rayDir, int nodeId, float maxDistance, const std::function<bool(entt::entity)>& filter, float& distance);
        void overlapBVHNode(glm::vec3 pos, glm::quat ori, Geometry geometry, Bounds bounds, int nodeId, int filter, std::vector<OverlapHit>& out);
        void overlapMtdBVHNode(glm::vec3 pos, glm::quat ori, Geometry geometry, Bounds bounds, int nodeId, std::vector<OverlapMtdHit>& out);

    public:
        PHYSECS_API Scene (entt::registry& registry);

        PHYSECS_API void setNumSubSteps(int numSubSteps);
        PHYSECS_API void setNumIterations(int numIterations);
        PHYSECS_API void setGravity(float gravity);
        PHYSECS_API void simulate(float timeStep);
        PHYSECS_API entt::entity raycastClosest(glm::vec3 rayOrig, glm::vec3 rayDir, float maxDistance, glm::vec3* hitPos = nullptr);
        PHYSECS_API entt::entity raycastClosest(glm::vec3 rayOrig, glm::vec3 rayDir, float maxDistance, const std::function<bool(entt::entity)>& filter, glm::vec3* hitPos = nullptr);
        PHYSECS_API std::vector<OverlapHit> overlap(glm::vec3 pos, glm::quat ori, Geometry geometry, int filter);
        PHYSECS_API std::vector<OverlapMtdHit> overlapWithMinTranslationalDistance(glm::vec3 pos, glm::quat ori, Geometry geometry);
        template<typename T>
        T* createJoint(entt::entity entity0, glm::vec3 anchor0Pos, glm::quat anchor0Or, entt::entity entity1, glm::vec3 anchor1Pos, glm::quat anchor1Or) {
            T* joint = new T(entity0, anchor0Pos, anchor0Or, entity1, anchor1Pos, anchor1Or);
            joint->update(registry);
            joints.push_back(joint);
            nonCollidingPairs.insert(entity0 < entity1 ? EntityPair{ entity0, entity1} : EntityPair{ entity1, entity0 });
            return joint;
        }
        PHYSECS_API void destroyJoint(Joint* joint);
        PHYSECS_API void clearColliders(entt::entity entity);
        PHYSECS_API void addCollider(entt::entity entity, const Collider& collider);
        PHYSECS_API void addOnTriggerEnterCallback(OnTriggerEnterListener* callback);
        PHYSECS_API void addOnTriggerExitCallback(OnTriggerExitListener* callback);
        PHYSECS_API void removeOnTriggerEnterCallback(OnTriggerEnterListener* callback);
        PHYSECS_API void removeOnTriggerExitCallback(OnTriggerExitListener* callback);
        PHYSECS_API void setContactFilter(ContactType (*filter)(bool, int, bool, int));
        PHYSECS_API entt::registry& getRegistry();
        PHYSECS_API const std::vector<glm::vec3>& getContactPoints();
        PHYSECS_API const std::vector<BVHNode>& getBVH();
        PHYSECS_API const int getBHVRootId();
        PHYSECS_API ~Scene();
    };
}
