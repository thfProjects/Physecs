#include "Physecs.h"
#include <chrono>
#include <execution>

#include "BoundsUtil.h"
#include "Collision.h"
#include "Constraint1D.h"
#include <glm/gtx/matrix_cross_product.hpp>
#include "Components.h"
#include "BVH.h"
#include "ContactConstraints.h"
#include "MathUtil.h"
#include "Overlap.h"
#include "Raycast.h"
#include "ContactManifold.h"
#include "Profiling.h"

const char* frameName = "Solver";

physecs::ContactType physecs::defaultContactFilter(bool isTrigger0, int data0, bool isTrigger1, int data1) {
    if (isTrigger0 || isTrigger1) return TRIGGER;
    return COLLISION;
}

void physecs::Scene::onRigidBodyCreate(entt::registry& registry, entt::entity entity) {
    auto& transform = registry.get<TransformComponent>(entity);
    auto& col = registry.get<RigidBodyCollisionComponent>(entity);
    auto dynamic = registry.try_get<RigidBodyDynamicComponent>(entity);
    bool isDynamic = dynamic ? !dynamic->isKinematic : false;
    for (int i = 0; i < col.colliders.size(); ++i) {
        auto& collider = col.colliders[i];
        auto bounds = getBounds(transform.position + transform.orientation * collider.position, transform.orientation * collider.orientation, collider.geometry);
        int nodeId = bvh.insert(entity, i, bounds);
        colToBroadPhaseEntry[{ entity, i }] = broadPhaseEntries.size();
        broadPhaseEntries.push_back({ entity, i, bounds, nodeId, true, collider.enableSimulation, isDynamic });
    }
}

void physecs::Scene::onRigidBodyDelete(entt::registry& registry, entt::entity entity) {
    auto& col = registry.get<RigidBodyCollisionComponent>(entity);
    for (int i = 0; i < col.colliders.size(); ++i) {
        int broadPhaseId = colToBroadPhaseEntry[{ entity, i }];
        auto& broadPhaseEntry = broadPhaseEntries[broadPhaseId];
        bvh.remove(broadPhaseEntry.nodeId);
        broadPhaseEntries[broadPhaseId] = broadPhaseEntries.back();
        colToBroadPhaseEntry[{broadPhaseEntries[broadPhaseId].entity, broadPhaseEntries[broadPhaseId].colliderIndex}] = broadPhaseId;
        broadPhaseEntries.pop_back();
    }
}

void physecs::Scene::onRigidBodyMove(entt::registry& registry, entt::entity entity) {
    if (!registry.any_of<RigidBodyCollisionComponent>(entity)) return;
    updateBounds(entity);
}

void physecs::Scene::onDynamicCreate(entt::registry &registry, entt::entity entity) {
    if (!registry.any_of<RigidBodyCollisionComponent>(entity)) return;

    auto& col = registry.get<RigidBodyCollisionComponent>(entity);
    auto& dynamic = registry.get<RigidBodyDynamicComponent>(entity);
    for (int i = 0; i < col.colliders.size(); ++i) {
        int broadPhaseId = colToBroadPhaseEntry[{ entity, i }];
        auto& broadPhaseEntry = broadPhaseEntries[broadPhaseId];
        broadPhaseEntry.isDynamic = !dynamic.isKinematic;
    }
}

void physecs::Scene::onDynamicDelete(entt::registry &registry, entt::entity entity) {
    if (!registry.any_of<RigidBodyCollisionComponent>(entity)) return;

    auto& col = registry.get<RigidBodyCollisionComponent>(entity);
    for (int i = 0; i < col.colliders.size(); ++i) {
        int broadPhaseId = colToBroadPhaseEntry[{ entity, i }];
        auto& broadPhaseEntry = broadPhaseEntries[broadPhaseId];
        broadPhaseEntry.isDynamic = false;
    }
}

void physecs::Scene::updateBounds(entt::entity entity) {
    auto& transform = registry.get<TransformComponent>(entity);
    auto& col = registry.get<RigidBodyCollisionComponent>(entity);
    for (int i = 0; i < col.colliders.size(); ++i) {
        auto& collider = col.colliders[i];
        int broadPhaseId = colToBroadPhaseEntry[{ entity, i }];
        auto& broadPhaseEntry = broadPhaseEntries[broadPhaseId];
        broadPhaseEntry.bounds = getBounds(transform.position + transform.orientation * collider.position, transform.orientation * collider.orientation, collider.geometry);
        broadPhaseEntry.bounds.addMargin(glm::vec3(0.01f));
        broadPhaseEntry.nodeDirty = true;
    }
}

physecs::Scene::Scene(entt::registry& registry) : registry(registry) {
    registry.on_construct<RigidBodyCollisionComponent>().connect<&Scene::onRigidBodyCreate>(this);
    registry.on_destroy<RigidBodyCollisionComponent>().connect<&Scene::onRigidBodyDelete>(this);
    registry.on_update<TransformComponent>().connect<&Scene::onRigidBodyMove>(this);
    registry.on_construct<RigidBodyDynamicComponent>().connect<&Scene::onDynamicCreate>(this);
    registry.on_destroy<RigidBodyDynamicComponent>().connect<&Scene::onDynamicDelete>(this);
}

void physecs::Scene::setNumSubSteps(int numSubSteps) {
    this->numSubSteps = numSubSteps;
}

void physecs::Scene::setNumIterations(int numIterations) {
    this->numIterations = numIterations;
}

void physecs::Scene::setGravity(float gravity) {
    this->g = gravity;
}

void physecs::Scene::simulate(float timeStep) {
    PhysecsFrameMarkStart(frameName);
    PhysecsZoneScoped;

    //SAP broad-phase
    PhysecsZoneN(broadPhase, "BroadPhase", true);
    for (int i = 1; i < broadPhaseEntries.size(); ++i) {
        auto broadPhaseEntry = broadPhaseEntries[i];
        int j = i;
        while (j > 0 && broadPhaseEntries[j-1].bounds.min.x > broadPhaseEntry.bounds.min.x) {
            broadPhaseEntries[j] = broadPhaseEntries[j-1];
            colToBroadPhaseEntry[{broadPhaseEntries[j].entity, broadPhaseEntries[j].colliderIndex}] = j;
            --j;
        }
        if (j < i) {
            broadPhaseEntries[j] = broadPhaseEntry;
            colToBroadPhaseEntry[{broadPhaseEntries[j].entity, broadPhaseEntries[j].colliderIndex}] = j;
        }
    }
    potentialContacts.clear();
    for (int i = 0; i < broadPhaseEntries.size() - 1; ++i) {
        auto& entry0 = broadPhaseEntries[i];

        if (!entry0.enableSimulation) continue;

        for (int j = i + 1; j < broadPhaseEntries.size(); ++j) {
            auto& entry1 = broadPhaseEntries[j];

            if (!entry1.enableSimulation) continue;

            if (entry0.entity == entry1.entity) continue;

            if (!(entry0.isDynamic || entry1.isDynamic)) continue;

            auto& bounds0 = entry0.bounds;
            auto& bounds1 = entry1.bounds;

            if (bounds0.max.x < bounds1.min.x) break;
            if (bounds0.max.y < bounds1.min.y || bounds0.min.y > bounds1.max.y) continue;
            if (bounds0.max.z < bounds1.min.z || bounds0.min.z > bounds1.max.z) continue;

            entt::entity entity0, entity1;
            int colliderIndex0, colliderIndex1;
            if (entry0.entity < entry1.entity) {
                entity0 = entry0.entity;
                colliderIndex0 = entry0.colliderIndex;
                entity1 = entry1.entity;
                colliderIndex1 = entry1.colliderIndex;
            }else {
                entity0 = entry1.entity;
                colliderIndex0 = entry1.colliderIndex;
                entity1 = entry0.entity;
                colliderIndex1 = entry0.colliderIndex;
            }

            potentialContacts.push_back({ entity0, colliderIndex0, entity1, colliderIndex1 });
        }
    }
    PhysecsZoneEnd(broadPhase);

    //narrow-phase
    PhysecsZoneN(narrowPhase, "NarrowPhase", true);
    contactConstraints.clear();
    triggerCacheTemp.clear();
    contactCacheTemp.clear();
    contactPoints.clear();
    for (auto contactPair : potentialContacts) {
        auto entity0 = contactPair.entity0;
        auto entity1 = contactPair.entity1;

        auto& col0 = registry.get<RigidBodyCollisionComponent>(entity0).colliders[contactPair.colliderIndex0];
        auto& col1 = registry.get<RigidBodyCollisionComponent>(entity1).colliders[contactPair.colliderIndex1];

        auto& transform0 = registry.get<TransformComponent>(entity0);
        auto& transform1 = registry.get<TransformComponent>(entity1);

        auto pos0 = transform0.position + transform0.orientation * col0.position;
        auto or0 = transform0.orientation * col0.orientation;

        auto pos1 = transform1.position + transform1.orientation * col1.position;
        auto or1 = transform1.orientation * col1.orientation;

        ContactType contactType = contactFilter(col0.isTrigger, col0.data, col1.isTrigger, col1.data);
        if (contactType == TRIGGER) {
            if (physecs::overlap(pos0, or0, col0.geometry, pos1, or1, col1.geometry)) {
                triggerCacheTemp.insert(contactPair);
                if (triggerCache.find(contactPair) == triggerCache.end()) {
                    for (auto callback : onTriggerEnterCallbacks) {
                        callback->onTriggerEnter(entity0, contactPair.colliderIndex0, entity1, contactPair.colliderIndex1);
                    }
                }
            }
            continue;
        }

        if (nonCollidingPairs.count({ entity0, entity1 })) continue;

        contactBuffer.clear();
        if (collision(pos0, or0, col0.geometry, pos1, or1, col1.geometry, contactBuffer)) {

            auto dynamic0 = registry.try_get<RigidBodyDynamicComponent>(contactPair.entity0);
            auto dynamic1 = registry.try_get<RigidBodyDynamicComponent>(contactPair.entity1);

            glm::vec3 com0(0), velocity0(0), angularVelocity0(0);
            if (dynamic0 && !dynamic0->isKinematic) {
                com0 = transform0.position + transform0.orientation * dynamic0->com;
                velocity0 = dynamic0->velocity;
                angularVelocity0 = dynamic0->angularVelocity;
            }

            glm::vec3 com1(0), velocity1(0), angularVelocity1(0);
            if (dynamic1 && !dynamic1->isKinematic) {
                com1 = transform1.position + transform1.orientation * dynamic1->com;
                velocity1 = dynamic1->velocity;
                angularVelocity1 = dynamic1->angularVelocity;
            }

            for (auto& collisionResult : contactBuffer) {
                if (!collisionResult.numPoints) continue;

                glm::vec3 n = collisionResult.normal;

                ContactManifoldData* prevContactData = contactCache.find({ contactPair, collisionResult.triangleIndex }) != contactCache.end() ? &contactCache.at({ contactPair, collisionResult.triangleIndex }) : nullptr;
                ContactManifoldData currContactData{ collisionResult.numPoints, {} };

                float friction = (col0.material.friction + col1.material.friction) * 0.5f;

                bool isSoft;
                float frequency;
                float damping;
                float restitution;
                if (col0.material.damping || col1.material.damping) {
                    //soft contact
                    isSoft = true;
                    if (col0.material.damping && col1.material.damping) {
                        frequency = glm::min(col0.material.restitution, col1.material.restitution);
                        damping = glm::min(col0.material.damping, col1.material.damping);
                    }
                    else if (col0.material.damping) {
                        frequency = col0.material.restitution;
                        damping = col0.material.damping;
                    }
                    else if (col1.material.damping) {
                        frequency = col1.material.restitution;
                        damping = col1.material.damping;
                    }
                    restitution = 0;
                }
                else {
                    //hard contact
                    isSoft = false;
                    frequency = 0;
                    damping = 0;
                    restitution = (col0.material.restitution + col1.material.restitution) * 0.5f;
                }

                ContactConstraints cc = { transform0, transform1, dynamic0, dynamic1, n, friction, isSoft, frequency, damping, collisionResult.numPoints, {}};

                for (int k = 0; k < collisionResult.numPoints; ++k) {
                    glm::vec3 r0 = collisionResult.points[k].position0 - com0;
                    glm::vec3 r1 = collisionResult.points[k].position1 - com1;

                    contactPoints.push_back(collisionResult.points[k].position0);
                    contactPoints.push_back(collisionResult.points[k].position1);

                    glm::vec3 relVelocity = velocity1 + glm::cross(angularVelocity1, r1) - velocity0 - glm::cross(angularVelocity0, r0);
                    float relNVelocity = glm::dot(relVelocity, n);

                    r0 = glm::inverse(transform0.orientation) * r0;
                    r1 = glm::inverse(transform1.orientation) * r1;

                    float targetVelocity;
                    bool prevContactFound = false;
                    if (prevContactData) {
                        for (int i = 0; i < prevContactData->numPoints; ++i) {
                            auto prevContact = prevContactData->contactPointData[i];
                            if (glm::distance(r0, prevContact.localPosition0) < 0.1) {
                                prevContactFound = true;
                                targetVelocity = prevContact.targetVelocity;
                                break;
                            }
                        }
                    }

                    if (!prevContactFound) {
                        targetVelocity = -restitution * relNVelocity;
                    }

                    //printf("triangle index: %d, prevContactFound: %d, normal: %f %f %f, targetVelocity: %f\n", collisionResult.triangleIndex, prevContactFound, n.x, n.y, n.z, targetVelocity);

                    currContactData.contactPointData[k] = { r0, targetVelocity };
                    cc.contactPointConstraints[k] = { r0, r1, glm::vec3(0), glm::vec3(0), glm::vec3(0), glm::vec3(0), glm::vec3(0), targetVelocity, 0, 0, 0 };
                }

                contactCacheTemp[{ contactPair, collisionResult.triangleIndex }] = currContactData;
                contactConstraints.push_back(cc);
            }
        }
    }
    PhysecsZoneEnd(narrowPhase);

    //create joint constraints
    PhysecsZoneN(ctx7, "create joint constraints", true);
    jointSolverDataBuffer.clear();
    jointConstraints.clear();
    for (auto& joint : joints) {
        auto solverData = joint->getSolverData(registry);
        jointSolverDataBuffer.push_back(solverData);
        for (int i = 0; i < solverData.numConstraints; ++i) {
            jointConstraints.emplace_back(solverData.transform0, solverData.transform1, solverData.dynamic0, solverData.dynamic1, glm::vec3(0), glm::vec3(0), glm::vec3(0), 0, 0, std::numeric_limits<float>::lowest(), std::numeric_limits<float>::max());
        }
    }
    PhysecsZoneEnd(ctx7);

    auto t1 = std::chrono::high_resolution_clock::now();

    float h = timeStep / numSubSteps;
    for (int m = 0; m < numSubSteps; ++m) {

        //update contact constraints
        PhysecsZoneN(ctx1, "update contact constraints", true);
        for (auto& contact : contactConstraints) {

            auto& n = contact.n;

            auto& transform0 = contact.transform0;
            auto& transform1 = contact.transform1;

            auto dynamic0 = contact.dynamic0;
            auto dynamic1 = contact.dynamic1;

            glm::vec3 com0(0), velocity0(0), angularVelocity0(0);
            if (dynamic0 && !dynamic0->isKinematic) {
                com0 = transform0.position + transform0.orientation * dynamic0->com;
                velocity0 = dynamic0->velocity;
                angularVelocity0 = dynamic0->angularVelocity;
            }

            glm::vec3 com1(0), velocity1(0), angularVelocity1(0);
            if (dynamic1 && !dynamic1->isKinematic) {
                com1 = transform1.position + transform1.orientation * dynamic1->com;
                velocity1 = dynamic1->velocity;
                angularVelocity1 = dynamic1->angularVelocity;
            }

            for (int k = 0; k < contact.numPoints; ++k) {
                auto& contactPoint = contact.contactPointConstraints[k];

                glm::vec3 r0 = transform0.orientation * contactPoint.r0;
                glm::vec3 r1 = transform1.orientation * contactPoint.r1;

                glm::vec3 contactPoint0 = com0 + r0;
                glm::vec3 contactPoint1 = com1 + r1;

                float cn = glm::dot(contactPoint1 - contactPoint0, n);

                glm::vec3 r0xn = glm::cross(r0, n);
                glm::vec3 r1xn = glm::cross(r1, n);

                glm::vec3 relVelocity = velocity1 + glm::cross(angularVelocity1, r1) - velocity0 - glm::cross(angularVelocity0, r0);
                float relNVelocity = glm::dot(relVelocity, n);

                //friction
                glm::vec3 t = relVelocity - relNVelocity * n;
                float tLen = glm::length(t);
                if (tLen) t = t / tLen;

                glm::vec3 r0xt = glm::cross(r0, t);
                glm::vec3 r1xt = glm::cross(r1, t);

                contactPoint.r0xn = r0xn;
                contactPoint.r1xn = r1xn;
                contactPoint.t = t;
                contactPoint.r0xt = r0xt;
                contactPoint.r1xt = r1xt;
                contactPoint.c = cn;
            }
        }
        PhysecsZoneEnd(ctx1);

        //update joint constraints
        PhysecsZoneN(ctx2, "update joint constraints", true);
        int index = 0;
        for (auto& jointSolverData : jointSolverDataBuffer) {
            JointWorldSpaceData worldSpaceData;
            jointSolverData.calculateWorldSpaceData(worldSpaceData);
            jointSolverData.makeConstraintsFunc(worldSpaceData, jointSolverData.additionalData, jointConstraints.data() + index);
            int endIndex = index + jointSolverData.numConstraints;
            for (int i = index; i < endIndex; ++i) {
                auto& jointConstraint = jointConstraints[i];
                jointConstraint.prepare();
                if (!(jointConstraint.flags & Constraint1D::SOFT) && glm::abs(jointConstraint.c) < 1e-4 && glm::abs(jointConstraint.totalLambda) < 10000) jointConstraint.warmStart();
            }
            index = endIndex;
        }
        PhysecsZoneEnd(ctx2);

        //integrate velocities and update world space inertia tensors
        PhysecsZoneN(ctx3, "Integrate velocities", true);
        for (auto [entity, transform, rigidDynamic] : registry.view<TransformComponent, RigidBodyDynamicComponent>().each()) {
            if (rigidDynamic.isKinematic) continue;

            glm::mat3 rot = glm::toMat3(transform.orientation);
            glm::mat3 invRot = glm::transpose(rot);

            rigidDynamic.velocity += h * glm::vec3(0, -g, 0);

            //gyro term
            glm::vec3 omegaLocal = invRot * rigidDynamic.angularVelocity;
            glm::mat3x3 I = glm::inverse(rigidDynamic.invInertiaTensor);
            glm::vec3 f = h * glm::cross(omegaLocal, I * omegaLocal);
            glm::mat3x3 J = I + h * (glm::matrixCross3(omegaLocal) * I - glm::matrixCross3(I * omegaLocal));
            omegaLocal = omegaLocal - solve33(J, f);

            rigidDynamic.angularVelocity = rot * omegaLocal;

            //update world space inertia tensor
            rigidDynamic.invInertiaTensorWorld = rot * rigidDynamic.invInertiaTensor * invRot;
        }
        PhysecsZoneEnd(ctx3);

        //constraint solve
        for (int i = 0; i < numIterations; ++i) {
            PhysecsZoneScopedN("constraint solve");
            for (auto& constraints : contactConstraints) {
                constraints.solve(true, h);
            }
            for (auto& constraint : jointConstraints) {
                constraint.solve(true, h);
            }
        }

        //integrate positions
        PhysecsZoneN(ctx4, "integrate positions", true);
        for (auto [entity, transform, rigidDynamic] : registry.view<TransformComponent, RigidBodyDynamicComponent>().each()) {
            if (rigidDynamic.isKinematic) continue;

            transform.position += h * rigidDynamic.velocity;

            glm::vec3 prevComWorld = transform.orientation * rigidDynamic.com;

            transform.orientation += h * glm::quat(0, rigidDynamic.angularVelocity) * transform.orientation / 2.f;
            transform.orientation = glm::normalize(transform.orientation);

            transform.position += prevComWorld - transform.orientation * rigidDynamic.com;
        }
        PhysecsZoneEnd(ctx4);

        //relaxation
        PhysecsZoneN(ctx5, "relaxation", true);
        for (auto& constraints : contactConstraints) {
            if (constraints.isSoft) continue;
            constraints.solve(false);
        }
        PhysecsZoneEnd(ctx5);
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> ms_double = t2 - t1;
    //printf("%f\n", ms_double.count());

    //cache triggers and contacts
    for (const auto& pair : triggerCache) {
        if (triggerCacheTemp.find(pair) == triggerCacheTemp.end()) {
            for (auto callback : onTriggerExitCallbacks) {
                callback->onTriggerExit(pair.entity0, pair.colliderIndex0, pair.entity1, pair.colliderIndex1);
            }
        }
    }
    triggerCache.swap(triggerCacheTemp);
    contactCache.swap(contactCacheTemp);

    //update bounds
    for (auto [entity, rigidDynamic] : registry.view<RigidBodyDynamicComponent>().each()) {
        if (rigidDynamic.isKinematic) continue;
        updateBounds(entity);
    }
    PhysecsFrameMarkEnd(frameName);
}

void physecs::Scene::updateBVH() {
    for (auto& broadPhaseEntry : broadPhaseEntries) {
        if (!broadPhaseEntry.nodeDirty) continue;
        bvh.update(broadPhaseEntry.nodeId, broadPhaseEntry.bounds);
        broadPhaseEntry.nodeDirty = false;
    }
}

entt::entity physecs::Scene::raycastClosestBVHNode(glm::vec3 rayOrig, glm::vec3 rayDir, int nodeId, float maxDistance, const std::function<bool(entt::entity)>& filter, float& distance) {
    const auto& node = bvh.getNodes()[nodeId];
    auto& bounds = node.bounds;
    if (intersectRayAABB(rayOrig, rayDir, bounds.min, bounds.max, distance)) {
        if (distance > maxDistance) return entt::null;
        if (node.isLeaf) {
            auto& transform = registry.get<TransformComponent>(node.leaf.entity);
            auto& collider = registry.get<RigidBodyCollisionComponent>(node.leaf.entity).colliders[node.leaf.colliderIndex];
            if (intersectRayGeometry(rayOrig, rayDir, transform.position + transform.orientation * collider.position, transform.orientation * collider.orientation, collider.geometry, distance)) {
                if (distance > maxDistance) return entt::null;
                if (filter && filter(node.leaf.entity)) return node.leaf.entity;
            }
            return entt::null;
        }

        float d1, d2;
        entt::entity e1 = raycastClosestBVHNode(rayOrig, rayDir, node.internal.left, maxDistance, filter, d1);
        entt::entity e2 = raycastClosestBVHNode(rayOrig, rayDir, node.internal.right, maxDistance, filter, d2);

        if (e1 != entt::null) {
            if (e2 != entt::null) {
                if (d1 < d2) {
                    distance = d1;
                    return e1;
                }
                distance = d2;
                return e2;
            }
            distance = d1;
            return e1;
        }

        if (e2 != entt::null) {
            distance = d2;
            return e2;
        }
    }

    return entt::null;
}

entt::entity physecs::Scene::raycastClosest(glm::vec3 rayOrig, glm::vec3 rayDir, float maxDistance, glm::vec3* hitPos) {
    updateBVH();
    float distance;
    entt::entity entity = raycastClosestBVHNode(rayOrig, rayDir, bvh.getRootId(), maxDistance, {}, distance);
    if (hitPos) *hitPos = rayOrig + rayDir * distance;
    return entity;
}

entt::entity physecs::Scene::raycastClosest(glm::vec3 rayOrig, glm::vec3 rayDir, float maxDistance, const std::function<bool(entt::entity)>& filter, glm::vec3 *hitPos) {
    updateBVH();
    float distance;
    entt::entity entity = raycastClosestBVHNode(rayOrig, rayDir, bvh.getRootId(), maxDistance, filter, distance);
    if (hitPos) *hitPos = rayOrig + rayDir * distance;
    return entity;
}

void physecs::Scene::overlapBVHNode(glm::vec3 pos, glm::quat ori, Geometry geometry, Bounds bounds, int nodeId, int filter, std::vector<OverlapHit>& out) {
    const auto& node = bvh.getNodes()[nodeId];
    if (intersects(bounds, node.bounds)) {
        if (node.isLeaf) {
            auto& transform = registry.get<TransformComponent>(node.leaf.entity);
            auto& collider = registry.get<RigidBodyCollisionComponent>(node.leaf.entity).colliders[node.leaf.colliderIndex];
            if ((!filter || collider.data & filter) && physecs::overlap(pos, ori, geometry, transform.position + transform.orientation * collider.position, transform.orientation * collider.orientation, collider.geometry)) {
                out.push_back({ node.leaf.entity, node.leaf.colliderIndex });
            }
        }
        else {
            overlapBVHNode(pos, ori, geometry, bounds, node.internal.left, filter, out);
            overlapBVHNode(pos, ori, geometry, bounds, node.internal.right, filter, out);
        }
    }
}

std::vector<physecs::OverlapHit> physecs::Scene::overlap(glm::vec3 pos, glm::quat ori, Geometry geometry, int filter) {
    updateBVH();
    auto bounds = getBounds(pos, ori, geometry);
    std::vector<OverlapHit> out;
    overlapBVHNode(pos, ori, geometry, bounds, bvh.getRootId(), filter, out);
    return out;
}

void physecs::Scene::overlapMtdBVHNode(glm::vec3 pos, glm::quat ori, Geometry geometry, Bounds bounds, int nodeId, std::vector<OverlapMtdHit>& out) {
    const auto& node = bvh.getNodes()[nodeId];
    if (intersects(bounds, node.bounds)) {
        if (node.isLeaf) {
            auto& transform = registry.get<TransformComponent>(node.leaf.entity);
            auto& collider = registry.get<RigidBodyCollisionComponent>(node.leaf.entity).colliders[node.leaf.colliderIndex];
            std::vector<ContactManifold> collisionResults;
            if (collision(transform.position + transform.orientation * collider.position, transform.orientation * collider.orientation, collider.geometry, pos, ori, geometry, collisionResults)) {
                for (auto& collisionResult : collisionResults) {
                    if (!collisionResult.numPoints) continue;
                    float mtd = 0;
                    for (int i = 0; i < collisionResult.numPoints; i++) {
                        auto [contact0, contact1] = collisionResult.points[i];
                        float dist = glm::dot(contact0 - contact1, collisionResult.normal);
                        if (dist > mtd) {
                            mtd = dist;
                        }
                    }
                    out.push_back({ node.leaf.entity, node.leaf.colliderIndex, collisionResult.normal, mtd });
                }
            }
        }
        else {
            overlapMtdBVHNode(pos, ori, geometry, bounds, node.internal.left, out);
            overlapMtdBVHNode(pos, ori, geometry, bounds, node.internal.right, out);
        }
    }
}

std::vector<physecs::OverlapMtdHit> physecs::Scene::overlapWithMinTranslationalDistance(glm::vec3 pos, glm::quat ori, Geometry geometry) {
    updateBVH();
    auto bounds = getBounds(pos, ori, geometry);
    std::vector<OverlapMtdHit> out;
    overlapMtdBVHNode(pos, ori, geometry, bounds, bvh.getRootId(), out);
    return out;
}

void physecs::Scene::destroyJoint(Joint *joint) {
    auto iter = std::find(joints.begin(), joints.end(), joint);
    if (iter == joints.end()) return;
    joints.erase(iter);
    auto entity0 = joint->getEntity0();
    auto entity1 = joint->getEntity1();
    nonCollidingPairs.erase(entity0 < entity1 ? EntityPair{ entity0, entity1} : EntityPair{ entity1, entity0 });
    delete joint;
}

void physecs::Scene::clearColliders(entt::entity entity) {
    auto& col = registry.get<RigidBodyCollisionComponent>(entity);
    for (int i = 0; i < col.colliders.size(); ++i) {
        int broadPhaseId = colToBroadPhaseEntry[{ entity, i }];
        auto& broadPhaseEntry = broadPhaseEntries[broadPhaseId];
        bvh.remove(broadPhaseEntry.nodeId);
        broadPhaseEntries[broadPhaseId] = broadPhaseEntries.back();
        colToBroadPhaseEntry[{broadPhaseEntries[broadPhaseId].entity, broadPhaseEntries[broadPhaseId].colliderIndex}] = broadPhaseId;
        broadPhaseEntries.pop_back();
    }
    col.colliders.clear();
}

void physecs::Scene::addCollider(entt::entity entity, const Collider &collider) {
    auto& transform = registry.get<TransformComponent>(entity);
    auto& col = registry.get<RigidBodyCollisionComponent>(entity);
    auto dynamic = registry.try_get<RigidBodyDynamicComponent>(entity);
    bool isDynamic = dynamic ? !dynamic->isKinematic : false;

    int i = col.colliders.size();
    auto bounds = getBounds(transform.position + transform.orientation * collider.position, transform.orientation * collider.orientation, collider.geometry);
    int nodeId = bvh.insert(entity, i, bounds);
    colToBroadPhaseEntry[{ entity, i }] = broadPhaseEntries.size();
    broadPhaseEntries.push_back({ entity, i, bounds, nodeId, true, collider.enableSimulation, isDynamic });

    col.colliders.push_back(collider);
}

void physecs::Scene::setIsKinematic(entt::entity entity, bool isKinematic) {
    auto& dynamic = registry.get<RigidBodyDynamicComponent>(entity);
    dynamic.isKinematic = isKinematic;

    if (isKinematic) {
        dynamic.velocity = glm::vec3(0);
        dynamic.angularVelocity = glm::vec3(0);
    }

    if (!registry.any_of<RigidBodyCollisionComponent>(entity)) return;

    auto& col = registry.get<RigidBodyCollisionComponent>(entity);
    for (int i = 0; i < col.colliders.size(); ++i) {
        int broadPhaseId = colToBroadPhaseEntry[{ entity, i }];
        auto& broadPhaseEntry = broadPhaseEntries[broadPhaseId];
        broadPhaseEntry.isDynamic = !isKinematic;
    }
}

void physecs::Scene::addOnTriggerEnterCallback(OnTriggerEnterListener* callback) {
    onTriggerEnterCallbacks.push_back(callback);
}

void physecs::Scene::addOnTriggerExitCallback(OnTriggerExitListener* callback) {
    onTriggerExitCallbacks.push_back(callback);
}

void physecs::Scene::removeOnTriggerEnterCallback(OnTriggerEnterListener *callback) {
    onTriggerEnterCallbacks.erase(std::find(onTriggerEnterCallbacks.begin(), onTriggerEnterCallbacks.end(), callback));
}

void physecs::Scene::removeOnTriggerExitCallback(OnTriggerExitListener *callback) {
    onTriggerExitCallbacks.erase(std::find(onTriggerExitCallbacks.begin(), onTriggerExitCallbacks.end(), callback));
}

void physecs::Scene::setContactFilter(ContactType(*filter)(bool, int, bool, int)) {
    contactFilter = filter;
}

entt::registry & physecs::Scene::getRegistry() {
    return registry;
}

const std::vector<glm::vec3> & physecs::Scene::getContactPoints() {
    return contactPoints;
}

const std::vector<physecs::BVHNode> & physecs::Scene::getBVH() {
    return bvh.getNodes();
}

const int physecs::Scene::getBHVRootId() {
    return bvh.getRootId();
}

physecs::Scene::~Scene() {
    registry.on_construct<RigidBodyCollisionComponent>().disconnect<&Scene::onRigidBodyCreate>(this);
    registry.on_destroy<RigidBodyCollisionComponent>().disconnect<&Scene::onRigidBodyDelete>(this);
    registry.on_update<TransformComponent>().disconnect<&Scene::onRigidBodyMove>(this);
    registry.on_construct<RigidBodyDynamicComponent>().disconnect<&Scene::onDynamicCreate>(this);
    registry.on_destroy<RigidBodyDynamicComponent>().disconnect<&Scene::onDynamicDelete>(this);
}
