#include "BoundsUtil.h"
#include "Components.h"
#include "Transform.h"
#include <glm/gtx/quaternion.hpp>

physecs::Bounds physecs::getBounds(entt::registry& registry, entt::entity entity) {
    auto transform = registry.get<TransformComponent>(entity);
    auto col = registry.get<RigidBodyCollisionComponent>(entity);

    Bounds bounds;
    if (!col.colliders.empty()) {
        auto& first = col.colliders.front();
        bounds = getBounds(transform.position + transform.orientation * first.position, transform.orientation * first.orientation, first.geometry);
        for (int i = 1; i < col.colliders.size(); ++i) {
            auto& other = col.colliders[i];
            bounds = getUnion(bounds, getBounds(transform.position + transform.orientation * other.position, transform.orientation * other.orientation, other.geometry));
        }
    }

    return bounds;
}

physecs::Bounds physecs::getBounds(glm::vec3 pos, glm::quat or, const Geometry &geom) {
    Bounds bounds;
    switch (geom.type) {
        case SPHERE: return getBoundsSphere(pos, geom.sphere.radius);
        case CAPSULE: return getBoundsCapsule(pos, or, geom.capsule.halfHeight, geom.capsule.radius);
        case BOX: return getBoundsBox(pos, or, geom.box.halfExtents);
        case CONVEX_MESH: return getBoundsConvexMesh(pos, or, geom.convex.mesh, geom.convex.scale);
        case TRIANGLE_MESH: return getBoundsTriangleMesh(pos, or, geom.triangleMesh.mesh);
    }
    return bounds;
}

physecs::Bounds physecs::getBoundsSphere(glm::vec3 pos, float radius) {
    return { pos - glm::vec3(radius), pos + glm::vec3(radius) };
}

physecs::Bounds physecs::getBoundsCapsule(glm::vec3 pos, glm::quat or, float halfHeight, float radius) {
    glm::vec3 p0 = pos + or * glm::vec3(0, halfHeight, 0);
    glm::vec3 p1 = pos + or * glm::vec3(0, -halfHeight, 0);
    glm::vec3 min = glm::min(p0, p1) - radius;
    glm::vec3 max = glm::max(p0, p1) + radius;
    return { min, max };
}


physecs::Bounds physecs::getBoundsBox(glm::vec3 pos, glm::quat or, glm::vec3 halfExtents) {
    glm::mat3 u = glm::toMat3(or);
    glm::mat3 absU = glm::mat3(
        glm::abs(u[0]),
        glm::abs(u[1]),
        glm::abs(u[2])
    );

    glm::vec3 worldHalfExtents = absU * halfExtents;

    return { pos - worldHalfExtents, pos + worldHalfExtents };
}

physecs::Bounds physecs::getBoundsConvexMesh(glm::vec3 pos, glm::quat or, ConvexMesh *mesh, glm::vec3 scale) {
    glm::vec3 min(std::numeric_limits<float>().max()), max(std::numeric_limits<float>().lowest());
    for (auto& vertexPos : mesh->vertices) {
        auto transformedPos = pos + or * (scale * vertexPos);
        min = glm::min(min, transformedPos);
        max = glm::max(max, transformedPos);
    }
    return { min, max };
}

physecs::Bounds physecs::getBoundsTriangle(glm::vec3 a, glm::vec3 b, glm::vec3 c) {
    glm::vec3 min = glm::min(glm::min(a, b), c);
    glm::vec3 max = glm::max(glm::max(a, b), c);
    return { min, max };
}

physecs::Bounds physecs::getBoundsTriangleMesh(glm::vec3 pos, glm::quat or, TriangleMesh *mesh) {
    glm::vec3 min(std::numeric_limits<float>().max()), max(std::numeric_limits<float>().lowest());
    for (auto& vertex : mesh->vertices) {
        auto transformedPos = pos + or * vertex;
        min = glm::min(min, transformedPos);
        max = glm::max(max, transformedPos);
    }
    return { min, max };
}

bool physecs::intersects(const Bounds &a, const Bounds &b) {
    if (a.max[0] < b.min[0] || a.min[0] > b.max[0]) return false;
    if (a.max[1] < b.min[1] || a.min[1] > b.max[1]) return false;
    if (a.max[2] < b.min[2] || a.min[2] > b.max[2]) return false;
    return true;
}

physecs::Bounds physecs::getUnion(const Bounds &a, const Bounds &b) {
    return {
        {
            glm::min(a.min.x, b.min.x),
            glm::min(a.min.y, b.min.y),
            glm::min(a.min.z, b.min.z),
        },
        {
            glm::max(a.max.x, b.max.x),
            glm::max(a.max.y, b.max.y),
            glm::max(a.max.z, b.max.z),
        }
    };
}
