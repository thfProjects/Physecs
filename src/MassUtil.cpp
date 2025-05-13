#include "MassUtil.h"
#include <glm/gtx/matrix_operation.hpp>
#include <glm/gtx/quaternion.hpp>
#include <array>

glm::mat3 physecs::getInertiaSphere(float mass, float radius) {
    return glm::diagonal3x3(glm::vec3(2.f * mass * radius * radius / 5.f));
}

glm::mat3 physecs::getInertiaCapsule(float mass, float halfHeight, float radius) {
    float vc = 2 * radius * radius * glm::pi<float>() * halfHeight;
    float vs = 4  * glm::pow(radius, 3) * glm::pi<float>() / 3.f;
    float v = vc + vs;
    float mc = vc * mass / v;
    float ms = vs * mass / v;

    float ixx = mc * (halfHeight * halfHeight / 3.f + radius * radius / 4.f) + ms * (halfHeight * halfHeight + 3.f * halfHeight * radius / 4.f + 2 * radius * radius / 5.f);
    float iyy = mc * radius * radius / 2.f + ms * 2 * radius * radius / 5.f;
    return glm::diagonal3x3(glm::vec3(ixx, iyy, ixx));
}

glm::mat3 physecs::getInertiaBox(float mass, glm::vec3 halfExtents) {
    float m = mass / 12.f;
    float x = halfExtents.x * halfExtents.x;
    float y = halfExtents.y * halfExtents.y;
    float z = halfExtents.z * halfExtents.z;
    return glm::diagonal3x3(glm::vec3(m * (y + z), m * (x + z), m * (x + y)));
}

glm::mat3 physecs::getInertiaTetrahedron(float mass, const std::array<glm::vec3, 4> &v) {
    //calculated around origin

    glm::vec3 d(0);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j <= i; j++) {
            d += v[j] * v[i];
        }
    }

    float a = (d.y + d.z) / 10.f;
    float b = (d.x + d.z) / 10.f;
    float c = (d.x + d.y) / 10.f;

    float aPrim = 0, bPrim = 0, cPrim = 0;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float mult = i == j ? 2 : 1;
            aPrim += mult * v[i].y * v[j].z;
            bPrim += mult * v[i].x * v[j].z;
            cPrim += mult * v[i].x * v[j].y;
        }
    }

    aPrim /= 20.f;
    bPrim /= 20.f;
    cPrim /= 20.f;

    glm::mat3 I;

    I[0][0] = a;
    I[1][1] = b;
    I[2][2] = c;
    I[0][1] = -bPrim;
    I[1][0] = -bPrim;
    I[2][0] = -cPrim;
    I[0][2] = -cPrim;
    I[1][2] = -aPrim;
    I[2][1] = -aPrim;

    return mass * I;
}

void physecs::computeCOMAndInvInertiaTensor(const RigidBodyCollisionComponent &collisionComponent, float mass, glm::vec3 &com, glm::mat3 &invInertiaTensor) {
    com = glm::vec3(0);
    float totalVolume = 0;
    for (auto collider : collisionComponent.colliders) {
        if (collider.isTrigger) continue;
        switch (collider.geometry.type) {
            case SPHERE: {
                float volume = 4 * glm::pi<float>() * glm::pow(collider.geometry.sphere.radius, 3) / 3.f;
                com += collider.position * volume;
                totalVolume += volume;
            }
            break;
            case CAPSULE: {
                float volume = 4 * glm::pi<float>() * glm::pow(collider.geometry.capsule.radius, 3) / 3.f + collider.geometry.capsule.radius * collider.geometry.capsule.radius * glm::pi<float>() * collider.geometry.capsule.halfHeight * 2.f;
                com += collider.position * volume;
                totalVolume += volume;
            }
            break;
            case BOX: {
                float volume = collider.geometry.box.halfExtents.x * collider.geometry.box.halfExtents.y * collider.geometry.box.halfExtents.z;
                com += collider.position * volume;
                totalVolume += volume;
            }
            break;
            case CONVEX_MESH: {
                auto convex = collider.geometry.convex;

                glm::vec3 center(0);
                for (auto& vertex : convex.mesh->vertices) {
                    center += convex.scale * vertex;
                }
                center /= convex.mesh->vertices.size();

                for (auto& face : convex.mesh->faces) {
                    glm::vec3 v0 = convex.scale * convex.mesh->vertices[face.indices[0]];
                    for (int i = 1; i < face.indices.size() - 1; i++) {
                        //calculate volume and centroid of tetrahedron
                        glm::vec3 v1 = convex.scale * convex.mesh->vertices[face.indices[i]];
                        glm::vec3 v2 = convex.scale * convex.mesh->vertices[face.indices[i + 1]];

                        float volume = glm::abs(glm::dot(glm::cross(v0 - center, v1 - center), v2 - center)) / 6.f;
                        glm::vec3 centroid = (center + v0 + v1 + v2) / 4.f;

                        com += (collider.position + centroid) * volume;
                        totalVolume += volume;
                    }
                }
            }
        }

    }
    com /= totalVolume;

    glm::mat3 inertiaTensor(0);

    for (auto collider : collisionComponent.colliders) {
        if (collider.isTrigger) continue;
        switch (collider.geometry.type) {
            case SPHERE: {
                float volume = 4 * glm::pi<float>() * glm::pow(collider.geometry.sphere.radius, 3) / 3.f;
                float m = mass * volume / totalVolume;
                glm::mat3 localInertiaTensor = getInertiaSphere(m, collider.geometry.sphere.radius);
                glm::vec3 r = com - collider.position;
                inertiaTensor += localInertiaTensor + m * (glm::mat3(glm::length2(r)) - glm::outerProduct(r, r)); //parallel axis theorem
            }
            break;
            case CAPSULE: {
                float volume = 4 * glm::pi<float>() * glm::pow(collider.geometry.capsule.radius, 3) / 3.f + collider.geometry.capsule.radius * collider.geometry.capsule.radius * glm::pi<float>() * collider.geometry.capsule.halfHeight * 2.f;
                float m = mass * volume / totalVolume;
                glm::mat3 localInertiaTensor = getInertiaCapsule(m, collider.geometry.capsule.halfHeight, collider.geometry.capsule.radius);
                glm::mat3 rot = glm::toMat3(collider.orientation);
                glm::vec3 r = com - collider.position;
                inertiaTensor += rot * localInertiaTensor * glm::transpose(rot) + m * (glm::mat3(glm::length2(r)) - glm::outerProduct(r, r)); //parallel axis theorem
            }
            break;
            case BOX: {
                float volume = collider.geometry.box.halfExtents.x * collider.geometry.box.halfExtents.y * collider.geometry.box.halfExtents.z;
                float m = mass * volume / totalVolume;
                glm::mat3 localInertiaTensor = getInertiaBox(m, collider.geometry.box.halfExtents);
                glm::mat3 rot = glm::toMat3(collider.orientation);
                glm::vec3 r = com - collider.position;
                inertiaTensor += rot * localInertiaTensor * glm::transpose(rot) + m * (glm::mat3(glm::length2(r)) - glm::outerProduct(r, r)); //parallel axis theorem
            }
            break;
            case CONVEX_MESH: {
                auto convex = collider.geometry.convex;

                glm::vec3 center(0);
                for (auto& vertex : convex.mesh->vertices) {
                    center += convex.scale * vertex;
                }
                center /= convex.mesh->vertices.size();

                for (auto& face : convex.mesh->faces) {
                    glm::vec3 v0 = convex.scale * convex.mesh->vertices[face.indices[0]];
                    for (int i = 1; i < face.indices.size() - 1; i++) {
                        glm::vec3 v1 = convex.scale * convex.mesh->vertices[face.indices[i]];
                        glm::vec3 v2 = convex.scale * convex.mesh->vertices[face.indices[i + 1]];

                        float volume = glm::abs(glm::dot(glm::cross(v0 - center, v1 - center), v2 - center)) / 6.f;
                        float m = mass * volume / totalVolume;
                        glm::vec3 d = collider.position - com;
                        inertiaTensor += getInertiaTetrahedron(m, { center + d, v0 + d, v1 + d, v2 + d });
                    }
                }
            }
        }
    }

    invInertiaTensor = glm::inverse(inertiaTensor);
}

void physecs::setMassProps(RigidBodyDynamicComponent &dynamicComponent, const RigidBodyCollisionComponent &collisionComponent, float mass) {
    dynamicComponent.invMass = 1.f / mass;
    glm::vec3 com;
    glm::mat3 invInertiaTensor;
    computeCOMAndInvInertiaTensor(collisionComponent, mass, com, invInertiaTensor);
    dynamicComponent.com = com;
    dynamicComponent.invInertiaTensor = invInertiaTensor;
}
