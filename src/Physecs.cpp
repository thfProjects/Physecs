#include "Physecs.h"
#include <chrono>
#include <ranges>
#include "Transform.h"
#include "BoundsUtil.h"
#include "Collision.h"
#include <glm/gtx/matrix_cross_product.hpp>
#include <glm/gtx/matrix_operation.hpp>

#include "Components.h"
#include "BVH.h"
#include "ContactConstraints.h"
#include "MathUtil.h"
#include "Overlap.h"
#include "Raycast.h"
#include "ContactManifold.h"
#include "Profiling.h"
#include "SolverData.h"

const char* frameName = "Solver";

physecs::DebugDrawContext debugDrawContext;

physecs::ContactType physecs::defaultContactFilter(bool isTrigger0, int data0, bool isTrigger1, int data1) {
    if (isTrigger0 || isTrigger1) return TRIGGER;
    return COLLISION;
}

void physecs::Scene::onRigidBodyCreate(const entt::registry& registry, entt::entity entity) {
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

void physecs::Scene::onRigidBodyDelete(const entt::registry& registry, entt::entity entity) {
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

void physecs::Scene::onRigidBodyMove(const entt::registry& registry, entt::entity entity) {
    if (!registry.any_of<RigidBodyCollisionComponent>(entity)) return;
    updateBounds(entity);
}

void physecs::Scene::onDynamicCreate(const entt::registry &registry, entt::entity entity) {
    if (!registry.any_of<RigidBodyCollisionComponent>(entity)) return;

    auto& col = registry.get<RigidBodyCollisionComponent>(entity);
    auto& dynamic = registry.get<RigidBodyDynamicComponent>(entity);
    for (int i = 0; i < col.colliders.size(); ++i) {
        int broadPhaseId = colToBroadPhaseEntry[{ entity, i }];
        auto& broadPhaseEntry = broadPhaseEntries[broadPhaseId];
        broadPhaseEntry.isDynamic = !dynamic.isKinematic;
    }
}

void physecs::Scene::onDynamicDelete(const entt::registry &registry, entt::entity entity) {
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

physecs::Scene::Scene(entt::registry& registry, int numThreads) : registry(registry), threadPool(numThreads) {
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

    debugDrawContext.clear();

    auto& entities = registry.storage<RigidBodyDynamicComponent>();
    auto* rigidBodies = entities.raw() ? *entities.raw() : nullptr;

    // partition rigid bodies so that all non-kinematic ones are contiguous at the start
    int numDynamicBodies = 0;
    for (int i = 0; i < entities.size(); ++i) {
        if (rigidBodies[i].isKinematic) continue;
        if (i != numDynamicBodies) entities.swap_elements(entities.at(i), entities.at(numDynamicBodies));
        ++numDynamicBodies;
    }

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
    threadPool.parallelFor(potentialContacts.size(), [this, rigidBodies](int index){
        PhysecsZoneScopedN("handle potential contact");
        auto& contactPair = potentialContacts[index];

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
                std::unique_lock lock(triggerMutex);
                triggerCacheTemp.insert(contactPair);
            }
            return;
        }

        if (nonCollidingPairs.contains({ entity0, entity1 })) return;

        thread_local std::vector<ContactManifold> contactBuffer;
        contactBuffer.clear();
        if (collision(pos0, or0, col0.geometry, pos1, or1, col1.geometry, contactBuffer)) {

            auto dynamic0 = registry.try_get<RigidBodyDynamicComponent>(contactPair.entity0);
            auto dynamic1 = registry.try_get<RigidBodyDynamicComponent>(contactPair.entity1);

            glm::vec3 com0(0), velocity0(0), angularVelocity0(0);
            glm::quat worldToCom0 = glm::quat(1, 0, 0, 0);
            if (dynamic0 && !dynamic0->isKinematic) {
                com0 = transform0.position + transform0.orientation * dynamic0->massProps.com;
                velocity0 = dynamic0->velocity;
                angularVelocity0 = dynamic0->angularVelocity;
                worldToCom0 = glm::conjugate(transform0.orientation * dynamic0->massProps.principalAxes);
            }

            glm::vec3 com1(0), velocity1(0), angularVelocity1(0);
            glm::quat worldToCom1 = glm::quat(1, 0, 0, 0);
            if (dynamic1 && !dynamic1->isKinematic) {
                com1 = transform1.position + transform1.orientation * dynamic1->massProps.com;
                velocity1 = dynamic1->velocity;
                angularVelocity1 = dynamic1->angularVelocity;
                worldToCom1 = glm::conjugate(transform1.orientation * dynamic1->massProps.principalAxes);
            }

            float friction = (col0.material.friction + col1.material.friction) * 0.5f;

            bool isSoft;
            float stiffness;
            float damping;
            float restitution;
            if (col0.material.damping || col1.material.damping) {
                //soft contact
                isSoft = true;
                if (col0.material.damping && col1.material.damping) {
                    stiffness = glm::min(col0.material.restitution, col1.material.restitution);
                    damping = glm::min(col0.material.damping, col1.material.damping);
                }
                else if (col0.material.damping) {
                    stiffness = col0.material.restitution;
                    damping = col0.material.damping;
                }
                else if (col1.material.damping) {
                    stiffness = col1.material.restitution;
                    damping = col1.material.damping;
                }
                restitution = 0;
            }
            else {
                //hard contact
                isSoft = false;
                stiffness = 0;
                damping = 0;
                restitution = (col0.material.restitution + col1.material.restitution) * 0.5f;
            }

            BodyId b0 = dynamic0 && !dynamic0->isKinematic ? dynamic0 - rigidBodies : INVALID_BODY_ID;
            BodyId b1 = dynamic1 && !dynamic1->isKinematic ? dynamic1 - rigidBodies : INVALID_BODY_ID;

            for (auto& collisionResult : contactBuffer) {
                if (!collisionResult.numPoints) continue;

                glm::vec3 n = collisionResult.normal;

                ContactManifoldData* prevContactData = contactCache.contains({ contactPair, collisionResult.triangleIndex }) ? &contactCache.at({ contactPair, collisionResult.triangleIndex }) : nullptr;
                ContactManifoldData currContactData{ collisionResult.numPoints, {} };

                ContactConstraints cc = {  nullptr, b0, b1, 0.f, 0.f, n, friction, isSoft, stiffness, damping, collisionResult.numPoints, {}};

                glm::vec3 frictionAnchor0 = glm::vec3(0), frictionAnchor1 = glm::vec3(0);

                for (int k = 0; k < collisionResult.numPoints; ++k) {
                    glm::vec3 r0 = collisionResult.points[k].position0 - com0;
                    glm::vec3 r1 = collisionResult.points[k].position1 - com1;

#ifdef DEBUG_CONTACT_POINTS
                    {
                        std::unique_lock lock(debugContactsMutex);
                        contactPoints.push_back(collisionResult.points[k].position0);
                        contactPoints.push_back(collisionResult.points[k].position1);
                    }
#endif

                    glm::vec3 relVelocity = velocity1 + glm::cross(angularVelocity1, r1) - velocity0 - glm::cross(angularVelocity0, r0);
                    float relNVelocity = glm::dot(relVelocity, n);

                    r0 = worldToCom0 * r0;
                    r1 = worldToCom1 * r1;

                    float targetVelocity;
                    float totalLambda;
                    bool prevContactFound = false;
                    if (prevContactData) {
                        for (int i = 0; i < prevContactData->numPoints; ++i) {
                            auto prevContact = prevContactData->contactPointData[i];
                            if (glm::distance(r0, prevContact.position0) < 0.1) {
                                prevContactFound = true;
                                targetVelocity = prevContact.targetVelocity;
                                totalLambda = prevContact.totalLambda;
                                break;
                            }
                        }
                    }

                    if (!prevContactFound) {
                        targetVelocity = -restitution * relNVelocity;
                        totalLambda = 0.f;
                    }

                    currContactData.contactPointData[k] = { r0, targetVelocity };
                    cc.contactPointConstraints[k] = { r0, r1, glm::vec3(0), glm::vec3(0), targetVelocity, 0, totalLambda };

                    frictionAnchor0 += r0;
                    frictionAnchor1 += r1;
                }

                frictionAnchor0 /= collisionResult.numPoints;
                frictionAnchor1 /= collisionResult.numPoints;

                cc.frictionConstraints = { frictionAnchor0, frictionAnchor1, glm::vec3(0), glm::vec3(0), glm::vec3(0), 0 };

                for (int k = 0; k < collisionResult.numPoints; ++k) {
                    float distToFrictionAnchor = glm::distance(cc.contactPointConstraints[k].r0, frictionAnchor0);
                    cc.contactPointConstraints[k].distToFrictionAnchor = distToFrictionAnchor;
                }

                std::unique_lock lock(collisionMutex);
                contactCacheTemp[{ contactPair, collisionResult.triangleIndex }] = currContactData;
                cc.contactManifoldData = &contactCacheTemp[{ contactPair, collisionResult.triangleIndex }];
                contactConstraints.push_back(cc);
            }
        }
    });
    PhysecsZoneEnd(narrowPhase);

    //create joint constraints
    PhysecsZoneN(ctx7, "create joint constraints", true);
    for (auto& [joints, jointSolverDataBuffer, jointConstraints] : jointGraph.colors) {
        jointSolverDataBuffer.clear();
        jointConstraints.clear();
        for (auto& joint : joints) {
            auto entity0 = joint->getEntity0();
            auto entity1 = joint->getEntity1();

            auto dynamic0 = registry.try_get<RigidBodyDynamicComponent>(entity0);
            auto dynamic1 = registry.try_get<RigidBodyDynamicComponent>(entity1);

            BodyId b0 = dynamic0 && !dynamic0->isKinematic ? dynamic0 - rigidBodies : INVALID_BODY_ID;
            BodyId b1 = dynamic1 && !dynamic1->isKinematic ? dynamic1 - rigidBodies : INVALID_BODY_ID;

            glm::quat localToCom0 = glm::conjugate(dynamic0->massProps.principalAxes);
            glm::quat localToCom1 = glm::conjugate(dynamic1->massProps.principalAxes);

            glm::vec3 r0 = localToCom0 * (dynamic0 && !dynamic0->isKinematic ? joint->getAnchor0Pos() - dynamic0->massProps.com : joint->getAnchor0Pos());
            glm::vec3 r1 = localToCom1 * (dynamic1 && !dynamic1->isKinematic ? joint->getAnchor1Pos() - dynamic1->massProps.com : joint->getAnchor1Pos());

            glm::mat3 u0 = glm::toMat3(localToCom0 * joint->getAnchor0Or());
            glm::mat3 u1 = glm::toMat3(localToCom1 * joint->getAnchor1Or());

            Constraint1DLayout constraintLayout(jointConstraints, b0, b1);
            auto [additionalData, makeConstraintsFunc] = joint->getSolverDesc(registry, constraintLayout);

            jointSolverDataBuffer.emplace_back(b0, b1, r0, r1, u0, u1, additionalData, makeConstraintsFunc);
        }
    }
    PhysecsZoneEnd(ctx7);

    velocityTemp.resize(numDynamicBodies);
    pseudoVelocityTemp.resize(numDynamicBodies);
    massTemp.resize(numDynamicBodies);
    transformTemp.resize(numDynamicBodies);

    // write to temp buffers
    for (int i = 0; i < numDynamicBodies; ++i) {
        auto& rigidDynamic = rigidBodies[i];

        auto& transform = registry.get<TransformComponent>(entities.at(i));

        glm::vec3 comWorld = transform.position + transform.orientation * rigidDynamic.massProps.com;
        glm::quat comToWorld = transform.orientation * rigidDynamic.massProps.principalAxes;

        velocityTemp[i].velocity = fromVec3(rigidDynamic.velocity);
        velocityTemp[i].angularVelocity = fromVec3(glm::conjugate(comToWorld) * rigidDynamic.angularVelocity);
        massTemp[i].invInertiaTensorAndMass = fromPacked(rigidDynamic.massProps.invInertiaDiag, rigidDynamic.massProps.invMass);
        transformTemp[i] = { comWorld, glm::toMat3(comToWorld) };
    }

    float h = timeStep / numSubSteps;
    for (int m = 0; m < numSubSteps; ++m) {

        //update contact constraints
        PhysecsZoneN(ctx1, "update contact constraints", true);
        for (auto& contact : contactConstraints) {

            auto& n = contact.n;

            glm::vec3 com0 = transformTemp[contact.b0].comWorld;
            glm::mat3& rot0 = transformTemp[contact.b0].worldRotation;
            glm::vec3 velocity0 = asVec3(velocityTemp[contact.b0].velocity);
            glm::vec3 angularVelocity0 = asVec3(velocityTemp[contact.b0].angularVelocity);

            glm::vec3 com1 = transformTemp[contact.b1].comWorld;
            glm::mat3& rot1 = transformTemp[contact.b1].worldRotation;
            glm::vec3 velocity1 = asVec3(velocityTemp[contact.b1].velocity);
            glm::vec3 angularVelocity1 = asVec3(velocityTemp[contact.b1].angularVelocity);

            glm::vec3 nLocal0 = multiplyTranspose(rot0, n);
            glm::vec3 nLocal1 = multiplyTranspose(rot1, n);

            for (int k = 0; k < contact.numPoints; ++k) {
                auto& contactPoint = contact.contactPointConstraints[k];

                float cn = glm::dot(com1 - com0, n) + glm::dot(contactPoint.r1, nLocal1) - glm::dot(contactPoint.r0, nLocal0);

                glm::vec3 r0xn = glm::cross(contactPoint.r0, nLocal0);
                glm::vec3 r1xn = glm::cross(contactPoint.r1, nLocal1);

                contactPoint.r0xn = r0xn;
                contactPoint.r1xn = r1xn;
                contactPoint.c = cn;
            }

            //friction

            auto& fc = contact.frictionConstraints;

            glm::vec3 relVelocity = velocity1 + rot1 * glm::cross(angularVelocity1, fc.r1) - velocity0 - rot0 * glm::cross(angularVelocity0, fc.r0);
            float relNVelocity = glm::dot(relVelocity, n);

            glm::vec3 t = relVelocity - relNVelocity * n;
            float tLen = glm::length(t);
            if (tLen) t = t / tLen;

            glm::vec3 r0xt = glm::cross(fc.r0, multiplyTranspose(rot0, t));
            glm::vec3 r1xt = glm::cross(fc.r1, multiplyTranspose(rot1, t));

            fc.t = t;
            fc.r0xt = r0xt;
            fc.r1xt = r1xt;
            fc.n0 = nLocal0;
            fc.n1 = nLocal1;
        }
        PhysecsZoneEnd(ctx1);

        //update joint constraints
        PhysecsZoneN(ctx2, "update joint constraints", true);
        for (auto& [_, jointSolverDataBuffer, jointConstraints] : jointGraph.colors) {
            Constraint1DWriter constraintWriter(jointConstraints);
            for (auto& jointSolverData : jointSolverDataBuffer) {
                glm::vec3 r0 = transformTemp[jointSolverData.b0].worldRotation * jointSolverData.r0;
                glm::vec3 r1 = transformTemp[jointSolverData.b1].worldRotation * jointSolverData.r1;
                JointWorldSpaceData wsData = {
                    transformTemp[jointSolverData.b0].comWorld + r0,
                    transformTemp[jointSolverData.b1].comWorld + r1,
                    r0,
                    r1,
                    transformTemp[jointSolverData.b0].worldRotation * jointSolverData.u0,
                    transformTemp[jointSolverData.b1].worldRotation * jointSolverData.u1,
                };
                constraintWriter.setContext({ &transformTemp[jointSolverData.b0].worldRotation, &transformTemp[jointSolverData.b1].worldRotation });
                jointSolverData.makeConstraintsFunc(wsData, jointSolverData.additionalData, constraintWriter);
            }
        }
        PhysecsZoneEnd(ctx2);

        //integrate velocities and update world space inertia tensors
        PhysecsZoneN(ctx3, "Integrate velocities", true);
        for (int i = 0; i < numDynamicBodies; ++i) {
            velocityTemp[i].velocity += h * glm::vec3(0, -g, 0);

            //gyro term, implicit euler in body space
            glm::vec3& w = asVec3(velocityTemp[i].angularVelocity);
            glm::vec3 I = 1.f /  asVec3(massTemp[i].invInertiaTensorAndMass);
            glm::vec3 d = h * glm::vec3(I.z - I.y, I.x - I.z, I.y - I.x);
            glm::vec3 f = glm::vec3(w.z * w.y, w.x * w.z, w.y * w.x) * d;
            glm::mat3 J = {
                I.x,        w.z * d.y,  w.y * d.z, // col0
                w.z * d.x,  I.y,        w.x * d.z, // col1
                w.y * d.x,  w.x * d.y,  I.z        // col2
            };
            w -= solve33(J, f);

            // clear pseudo velocities
            pseudoVelocityTemp[i] = {};
        }
        PhysecsZoneEnd(ctx3);

        //pre solve
        PhysecsZoneN(ctx8, "pre solve", true);
        for (auto& constraints : contactConstraints) {
            constraints.preSolve(massTemp.getData(), velocityTemp.getData());
        }
        for (auto& color : jointGraph.colors) {
            color.jointConstraints.preSolve(massTemp.getData(), velocityTemp.getData(), pseudoVelocityTemp.getData());
        }
        PhysecsZoneEnd(ctx8);

        // solve
        for (int i = 0; i < numIterations; ++i) {
            PhysecsZoneScopedN("constraint solve");
            for (auto& color : jointGraph.colors) {
                color.jointConstraints.solve(velocityTemp.getData(), h, true);
            }
            for (auto& constraints : contactConstraints) {
                constraints.solve(velocityTemp.getData(), true, h);
            }
        }

        //integrate positions
        PhysecsZoneN(ctx4, "integrate positions", true);
        for (int i = 0; i < numDynamicBodies; ++i) {
            float pseudoVelocityScale = pseudoVelocityTemp[i].constraintCount ? 1.f / pseudoVelocityTemp[i].constraintCount : 1.f;

            glm::quat deltaRotation = glm::normalize(glm::quat(1.f, 0.5f * (h * asVec3(velocityTemp[i].angularVelocity) + pseudoVelocityScale * asVec3(pseudoVelocityTemp[i].pseudoAngularVelocity))));
            transformTemp[i].comWorld += h * asVec3(velocityTemp[i].velocity) + pseudoVelocityScale * asVec3(pseudoVelocityTemp[i].pseudoVelocity);
            transformTemp[i].worldRotation = transformTemp[i].worldRotation * glm::toMat3(deltaRotation);
        }
        PhysecsZoneEnd(ctx4);

        //relaxation
        PhysecsZoneN(ctx5, "relaxation", true);
        for (auto& color : jointGraph.colors) {
            color.jointConstraints.solve(velocityTemp.getData(), h, false);
        }
        for (auto& constraints : contactConstraints) {
            if (constraints.isSoft) continue;
            constraints.solve(velocityTemp.getData(), false);
        }
        PhysecsZoneEnd(ctx5);
    }

    // write back from temp buffers
    for (int i = 0; i < numDynamicBodies; ++i) {
        auto& rigidDynamic = rigidBodies[i];

        auto& transform = registry.get<TransformComponent>(entities.at(i));

        glm::quat comToWorld = glm::normalize(glm::toQuat(transformTemp[i].worldRotation));

        rigidDynamic.velocity = asVec3(velocityTemp[i].velocity);
        rigidDynamic.angularVelocity = comToWorld * asVec3(velocityTemp[i].angularVelocity);

        transform.orientation = comToWorld * glm::conjugate(rigidDynamic.massProps.principalAxes);
        transform.position = transformTemp[i].comWorld - transform.orientation * rigidDynamic.massProps.com;
    }

    // cache accumulated joint impulses for next step
    for (auto& [joints, jointSolverDataBuffer, jointConstraints] : jointGraph.colors) {
        Constraint1DReader constraintReader(jointConstraints);
        for (auto& joint : joints) {
            joint->storeAccumulatedImpulses(constraintReader);
        }
    }

#ifdef DEBUG_CONTACT_FORCES
    for (auto& contacts : contactConstraints) {
        float totalForce = 0.f;
        for (int i = 0; i < contacts.numPoints; ++i) {
            totalForce += contacts.contactPointConstraints[i].totalLambda;
        }
        totalForce /= h;
        totalForce *= 0.1f;
        glm::vec3 start = contacts.transform0.position + contacts.transform0.orientation * contacts.frictionConstraints.r0;
        debugDrawContext.addLine(start, start - contacts.n * totalForce, Color::RED);
    }
#endif

    //cache triggers and contacts
    for (const auto& pair : triggerCacheTemp) {
        if (!triggerCache.contains(pair)) {
            for (auto callback : onTriggerEnterCallbacks) {
                callback->onTriggerEnter(pair.entity0, pair.colliderIndex0, pair.entity1, pair.colliderIndex1);
            }
        }
    }
    for (const auto& pair : triggerCache) {
        if (!triggerCacheTemp.contains(pair)) {
            for (auto callback : onTriggerExitCallbacks) {
                callback->onTriggerExit(pair.entity0, pair.colliderIndex0, pair.entity1, pair.colliderIndex1);
            }
        }
    }
    for (auto& contacts : contactConstraints) {
        for (int i = 0; i < contacts.numPoints; ++i) {
            contacts.contactManifoldData->contactPointData[i].totalLambda = contacts.contactPointConstraints[i].totalLambda;
        }
    }
    triggerCache.swap(triggerCacheTemp);
    contactCache.swap(contactCacheTemp);

    //update bounds
    for (int i = 0; i < numDynamicBodies; ++i) {
        updateBounds(entities.at(i));
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

entt::entity physecs::Scene::raycastClosestBVHNode(glm::vec3 rayOrig, glm::vec3 rayDir, int nodeId, float maxDistance, const std::function<bool(entt::entity)>& filter, float& distance) const {
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

void physecs::Scene::overlapBVHNode(glm::vec3 pos, glm::quat ori, Geometry geometry, const Bounds &bounds, int nodeId, int filter, std::vector<OverlapHit>& out) const {
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

std::vector<physecs::OverlapHit> physecs::Scene::overlap(glm::vec3 pos, glm::quat ori, const Geometry &geometry, int filter) {
    updateBVH();
    auto bounds = getBounds(pos, ori, geometry);
    std::vector<OverlapHit> out;
    overlapBVHNode(pos, ori, geometry, bounds, bvh.getRootId(), filter, out);
    return out;
}

void physecs::Scene::overlapMtdBVHNode(glm::vec3 pos, glm::quat ori, Geometry geometry, Bounds bounds, int nodeId, std::vector<OverlapMtdHit>& out) const {
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

std::vector<physecs::OverlapMtdHit> physecs::Scene::overlapWithMinTranslationalDistance(glm::vec3 pos, glm::quat ori, const Geometry &geometry) {
    updateBVH();
    auto bounds = getBounds(pos, ori, geometry);
    std::vector<OverlapMtdHit> out;
    overlapMtdBVHNode(pos, ori, geometry, bounds, bvh.getRootId(), out);
    return out;
}

void physecs::Scene::addJoint(Joint *joint) {
    const auto entity0 = joint->getEntity0();
    const auto entity1 = joint->getEntity1();

    nonCollidingPairs.insert(entity0 < entity1 ? EntityPair{ entity0, entity1} : EntityPair{ entity1, entity0 });

    auto& colors0 = jointGraph.bitsets[entity0];
    auto& colors1 = jointGraph.bitsets[entity1];

    const auto colorsUnion = colors0 | colors1;

    unsigned long i;
    _BitScanForward(&i, ~colorsUnion);

    colors0 |= 1 << i;
    colors1 |= 1 << i;

    jointGraph.colors[i].joints.push_back(joint);

    joint->setColor(i);
}

void physecs::Scene::destroyJoint(Joint *joint) {
    auto& joints = jointGraph.colors[joint->getColor()].joints;
    const auto iter = std::ranges::find(joints, joint);
    if (iter == joints.end()) return;
    joints.erase(iter);
    const auto entity0 = joint->getEntity0();
    const auto entity1 = joint->getEntity1();
    nonCollidingPairs.erase(entity0 < entity1 ? EntityPair{ entity0, entity1} : EntityPair{ entity1, entity0 });
    jointGraph.bitsets[entity0] &= ~(1 << joint->getColor());
    jointGraph.bitsets[entity1] &= ~(1 << joint->getColor());
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
    onTriggerEnterCallbacks.erase(std::ranges::find(onTriggerEnterCallbacks, callback));
}

void physecs::Scene::removeOnTriggerExitCallback(OnTriggerExitListener *callback) {
    onTriggerExitCallbacks.erase(std::ranges::find(onTriggerExitCallbacks, callback));
}

void physecs::Scene::setCanCollide(entt::entity entity0, entt::entity entity1, bool canCollide) {
    if (canCollide)
        nonCollidingPairs.erase(entity0 < entity1 ? EntityPair{ entity0, entity1} : EntityPair{ entity1, entity0 });
    else
        nonCollidingPairs.insert(entity0 < entity1 ? EntityPair{ entity0, entity1} : EntityPair{ entity1, entity0 });
}

void physecs::Scene::setContactFilter(ContactType(*filter)(bool, int, bool, int)) {
    contactFilter = filter;
}

entt::registry & physecs::Scene::getRegistry() const {
    return registry;
}

const std::vector<physecs::BVHNode> & physecs::Scene::getBVH() const {
    return bvh.getNodes();
}

int physecs::Scene::getBHVRootId() const {
    return bvh.getRootId();
}

const std::vector<glm::vec3> & physecs::Scene::getContactPoints() const {
    return contactPoints;
}

const physecs::DebugDrawContext & physecs::Scene::getDebugDrawContext() const {
    return debugDrawContext;
}

physecs::Scene::~Scene() {
    registry.on_construct<RigidBodyCollisionComponent>().disconnect<&Scene::onRigidBodyCreate>(this);
    registry.on_destroy<RigidBodyCollisionComponent>().disconnect<&Scene::onRigidBodyDelete>(this);
    registry.on_update<TransformComponent>().disconnect<&Scene::onRigidBodyMove>(this);
    registry.on_construct<RigidBodyDynamicComponent>().disconnect<&Scene::onDynamicCreate>(this);
    registry.on_destroy<RigidBodyDynamicComponent>().disconnect<&Scene::onDynamicDelete>(this);
}
