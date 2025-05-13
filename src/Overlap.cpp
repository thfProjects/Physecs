#include "Overlap.h"
#include <glm/gtx/quaternion.hpp>
#include "GeomUtil.h"
#include "GJK.h"

using namespace physecs;

bool overlapSphereSphere(glm::vec3 pos0, float radius0, glm::vec3 pos1, float radius1);
bool overlapCapsuleCapsule(glm::vec3 pos0, glm::quat or0, float halfHeight0, float radius0, glm::vec3 pos1, glm::quat or1, float halfHeight1, float radius1);
bool overlapBoxBox(glm::vec3 pos0, glm::quat or0, glm::vec3 halfExtents0, glm::vec3 pos1, glm::quat or1, glm::vec3 halfExtents1);
bool overlapConvexMeshConvexMesh(glm::vec3 pos0, glm::quat or0, ConvexMesh* mesh0, glm::vec3 scale0, glm::vec3 pos1, glm::quat or1, ConvexMesh* mesh1, glm::vec3 scale1);
bool overlapSphereCapsule(glm::vec3 pos0, float radius0, glm::vec3 pos1, glm::quat or1, float halfHeight1, float radius1);
bool overlapSphereBox(glm::vec3 pos0, float radius0, glm::vec3 pos1, glm::quat or1, glm::vec3 halfExtents1);
bool overlapSphereConvexMesh(glm::vec3 pos0, float radius0, glm::vec3 pos1, glm::quat or1, ConvexMesh* mesh1, glm::vec3 scale1);
bool overlapCapsuleBox(glm::vec3 pos0, glm::quat or0, float halfHeight0, float radius0, glm::vec3 pos1, glm::quat or1, glm::vec3 halfExtents1);
bool overlapCapsuleConvexMesh(glm::vec3 pos0, glm::quat or0, float halfHeight0, float radius0, glm::vec3 pos1, glm::quat or1, ConvexMesh* mesh1, glm::vec3 scale1);
bool overlapBoxConvexMesh(glm::vec3 pos0, glm::quat or0, glm::vec3 halfExtents0, glm::vec3 pos1, glm::quat or1, ConvexMesh* mesh1, glm::vec3 scale1);

bool overlapSphereSphere(glm::vec3 pos0, float radius0, glm::vec3 pos1, float radius1) {
    glm::vec3 d = pos1 - pos0;
    float radiusSum = radius0 + radius1;
    if (glm::dot(d, d) > radiusSum * radiusSum) return false;
    return true;
}

bool overlapCapsuleCapsule(glm::vec3 pos0, glm::quat or0, float halfHeight0, float radius0, glm::vec3 pos1, glm::quat or1, float halfHeight1, float radius1) {
    glm::vec3 up = glm::vec3(0, 1, 0);
    glm::vec3 v0 = or0 * up;
    glm::vec3 v1 = or1 * up;
    auto [point0, point1] = closestPointsBetweenSegments(pos0, v0, -halfHeight0, halfHeight0, pos1, v1, -halfHeight1, halfHeight1);
    glm::vec3 d = point1 - point0;
    float radiusSum = radius0 + radius1;
    if (glm::dot(d, d) > radiusSum * radiusSum) return false;
    return true;
}

bool overlapBoxBox(glm::vec3 pos0, glm::quat or0, glm::vec3 halfExtents0, glm::vec3 pos1, glm::quat or1, glm::vec3 halfExtents1) {
    float ra, rb, l, d;

    glm::mat3 r = glm::mat3();
    glm::mat3 absR = glm::mat3();

    //basis vectors
    glm::mat3 u0 = glm::toMat3(or0);
    glm::mat3 u1 = glm::toMat3(or1);

    glm::vec3 t = pos1 - pos0;
    t = glm::vec3(glm::dot(t, u0[0]), glm::dot(t, u0[1]), glm::dot(t, u0[2]));

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            r[i][j] = glm::dot(u0[i], u1[j]);
            absR[i][j] = glm::abs(r[i][j]);
        }
    }

    // Test axes L = A0, L = A1, L = A2
    for (int i = 0; i < 3; i++) {
        ra = halfExtents0[i];
        rb = halfExtents1[0] * absR[i][0] + halfExtents1[1] * absR[i][1] + halfExtents1[2] * absR[i][2];
        l = glm::abs(t[i]);
        d = l - ra - rb;
        if (d > 0) return false;
    }

    // Test axes L = B0, L = B1, L = B2
    for (int i = 0; i < 3; i++) {
        ra = halfExtents0[0] * absR[0][i] + halfExtents0[1] * absR[1][i] + halfExtents0[2] * absR[2][i];
        rb = halfExtents1[i];
        l = glm::abs(t[0] * r[0][i] + t[1] * r[1][i] + t[2] * r[2][i]);
        d = l - ra - rb;
        if (d > 0) return false;
    }

    // Test axis L = A0 x B0
    ra = halfExtents0[1] * absR[2][0] + halfExtents0[2] * absR[1][0];
    rb = halfExtents1[1] * absR[0][2] + halfExtents1[2] * absR[0][1];
    l = glm::abs(t[2] * r[1][0] - t[1] * r[2][0]);
    d = l - ra - rb;
    if (d > 0) return false;

    // Test axis L = A0 x B1
    ra = halfExtents0[1] * absR[2][1] + halfExtents0[2] * absR[1][1];
    rb = halfExtents1[0] * absR[0][2] + halfExtents1[2] * absR[0][0];
    l = glm::abs(t[2] * r[1][1] - t[1] * r[2][1]);
    d = l - ra - rb;
    if (d > 0) return false;

    // Test axis L = A0 x B2
    ra = halfExtents0[1] * absR[2][2] + halfExtents0[2] * absR[1][2];
    rb = halfExtents1[0] * absR[0][1] + halfExtents1[1] * absR[0][0];
    l = glm::abs(t[2] * r[1][2] - t[1] * r[2][2]);
    d = l - ra - rb;
    if (d > 0) return false;

    // Test axis L = A1 x B0
    ra = halfExtents0[0] * absR[2][0] + halfExtents0[2] * absR[0][0];
    rb = halfExtents1[1] * absR[1][2] + halfExtents1[2] * absR[1][1];
    l = glm::abs(t[0] * r[2][0] - t[2] * r[0][0]);
    d = l - ra - rb;
    if (d > 0) return false;

    // Test axis L = A1 x B1
    ra = halfExtents0[0] * absR[2][1] + halfExtents0[2] * absR[0][1];
    rb = halfExtents1[0] * absR[1][2] + halfExtents1[2] * absR[1][0];
    l = glm::abs(t[0] * r[2][1] - t[2] * r[0][1]);
    d = l - ra - rb;
    if (d > 0) return false;

    // Test axis L = A1 x B2
    ra = halfExtents0[0] * absR[2][2] + halfExtents0[2] * absR[0][2];
    rb = halfExtents1[0] * absR[1][1] + halfExtents1[1] * absR[1][0];
    l = glm::abs(t[0] * r[2][2] - t[2] * r[0][2]);
    d = l - ra - rb;
    if (d > 0) return false;

    // Test axis L = A2 x B0
    ra = halfExtents0[0] * absR[1][0] + halfExtents0[1] * absR[0][0];
    rb = halfExtents1[1] * absR[2][2] + halfExtents1[2] * absR[2][1];
    l = glm::abs(t[1] * r[0][0] - t[0] * r[1][0]);
    d = l - ra - rb;
    if (d > 0) return false;

    // Test axis L = A2 x B1
    ra = halfExtents0[0] * absR[1][1] + halfExtents0[1] * absR[0][1];
    rb = halfExtents1[0] * absR[2][2] + halfExtents1[2] * absR[2][0];
    l = glm::abs(t[1] * r[0][1] - t[0] * r[1][1]);
    d = l - ra - rb;
    if (d > 0) return false;

    // Test axis L = A2 x B2
    ra = halfExtents0[0] * absR[1][2] + halfExtents0[1] * absR[0][2];
    rb = halfExtents1[0] * absR[2][1] + halfExtents1[1] * absR[2][0];
    l = glm::abs(t[1] * r[0][2] - t[0] * r[1][2]);
    d = l - ra - rb;
    if (d > 0) return false;

    return true;
}

bool overlapConvexMeshConvexMesh(glm::vec3 pos0, glm::quat or0, ConvexMesh *mesh0, glm::vec3 scale0, glm::vec3 pos1, glm::quat or1, ConvexMesh *mesh1, glm::vec3 scale1) {
    return GJK<ConvexMeshSupportFunction, ConvexMeshSupportFunction>::gjk(ConvexMeshSupportFunction{ pos0, glm::toMat3(or0), mesh0->vertices, scale0 }, ConvexMeshSupportFunction{ pos1, glm::toMat3(or1), mesh1->vertices, scale1 }, pos1 - pos0);
}

bool overlapSphereCapsule(glm::vec3 pos0, float radius0, glm::vec3 pos1, glm::quat or1, float halfHeight1, float radius1) {
    glm::vec3 up = glm::vec3(0, 1, 0);
    glm::vec3 dir = or1 * up;
    glm::vec3 q = closestPointOnSegment(pos0, pos1, dir, -halfHeight1, halfHeight1);
    glm::vec3 v = q - pos0;
    float radiusSum = radius0 + radius1;
    if (glm::dot(v, v) > radiusSum * radiusSum) return false;
    return true;
}

bool overlapSphereBox(glm::vec3 pos0, float radius0, glm::vec3 pos1, glm::quat or1, glm::vec3 halfExtents1) {
    //basis vectors
    glm::mat3 u1 = glm::toMat3(or1);

    glm::vec3 d = pos0 - pos1;
    d = glm::vec3(glm::dot(d, u1[0]), glm::dot(d, u1[1]), glm::dot(d, u1[2]));

    //closest point on OBB to sphere
    glm::vec3 q = pos1 + u1 * glm::clamp(d, -halfExtents1, halfExtents1);

    glm::vec3 v = q - pos0;

    if (glm::dot(v, v) > radius0 * radius0) return false;
    return true;
}

bool overlapSphereConvexMesh(glm::vec3 pos0, float radius0, glm::vec3 pos1, glm::quat or1, ConvexMesh *mesh1, glm::vec3 scale1) {
    return GJK<SphereSupportFunction, ConvexMeshSupportFunction>::gjk(SphereSupportFunction{ pos0, radius0 }, ConvexMeshSupportFunction{ pos1, glm::toMat3(or1), mesh1->vertices, scale1 }, pos1 - pos0);
}

bool overlapCapsuleBox(glm::vec3 pos0, glm::quat or0, float halfHeight0, float radius0, glm::vec3 pos1, glm::quat or1, glm::vec3 halfExtents1) {
    //basis vectors
    glm::mat3 u1 = glm::toMat3(or1);

    glm::vec3 p = pos0 - pos1;
    p = glm::vec3(glm::dot(p, u1[0]), glm::dot(p, u1[1]), glm::dot(p, u1[2]));
    glm::vec3 dir = or0 * glm::vec3(0, 1, 0);
    glm::vec3 dirLc = glm::vec3(glm::dot(dir, u1[0]), glm::dot(dir, u1[1]), glm::dot(dir, u1[2]));

    float t;
    glm::vec3 q;

    float sqrDist = sqrDistSegmentAABB(p, dirLc, -halfHeight0, halfHeight0, halfExtents1, t, q);

    if (sqrDist >= radius0 * radius0)
        return false;

    return true;
}

bool overlapCapsuleConvexMesh(glm::vec3 pos0, glm::quat or0, float halfHeight0, float radius0, glm::vec3 pos1, glm::quat or1, ConvexMesh *mesh1, glm::vec3 scale1) {
    return GJK<CapsuleSupportFunction, ConvexMeshSupportFunction>::gjk(CapsuleSupportFunction{ pos0, or0 * glm::vec3(0, 1, 0), halfHeight0, radius0 }, ConvexMeshSupportFunction{ pos1, glm::toMat3(or1), mesh1->vertices, scale1 }, pos1 - pos0);
}

bool overlapBoxConvexMesh(glm::vec3 pos0, glm::quat or0, glm::vec3 halfExtents0, glm::vec3 pos1, glm::quat or1, ConvexMesh *mesh1, glm::vec3 scale1) {
    return GJK<BoxSupportFunction, ConvexMeshSupportFunction>::gjk(BoxSupportFunction{ pos0, glm::toMat3(or0), halfExtents0 }, ConvexMeshSupportFunction{ pos0, glm::toMat3(or1), mesh1->vertices, scale1 }, pos1 - pos0);
}

bool physecs::overlap(glm::vec3 pos0, glm::quat or0, Geometry& geom0, glm::vec3 pos1, glm::quat or1, Geometry& geom1) {
    switch (geom0.type) {
        case SPHERE:
            switch (geom1.type) {
                case SPHERE: return overlapSphereSphere(pos0, geom0.sphere.radius, pos1, geom1.sphere.radius);
                case CAPSULE: return overlapSphereCapsule(pos0, geom0.sphere.radius, pos1, or1, geom1.capsule.halfHeight, geom1.capsule.radius);
                case BOX: return overlapSphereBox(pos0, geom0.sphere.radius, pos1, or1, geom1.box.halfExtents);
                case CONVEX_MESH: return overlapSphereConvexMesh(pos0, geom0.sphere.radius, pos1, or1, geom1.convex.mesh, geom1.convex.scale);
            }
        case CAPSULE:
            switch (geom1.type) {
                case SPHERE: return overlapSphereCapsule(pos1, geom1.sphere.radius, pos0, or0, geom0.capsule.halfHeight, geom0.capsule.radius);
                case CAPSULE: return overlapCapsuleCapsule(pos0, or0, geom0.capsule.halfHeight, geom0.capsule.radius, pos1, or1, geom1.capsule.halfHeight, geom1.capsule.radius);
                case BOX: return overlapCapsuleBox(pos0, or0, geom0.capsule.halfHeight, geom0.capsule.radius, pos1, or1, geom1.box.halfExtents);
                case CONVEX_MESH: return overlapCapsuleConvexMesh(pos0, or0, geom0.capsule.halfHeight, geom0.capsule.radius, pos1, or1, geom1.convex.mesh, geom1.convex.scale);
            }
        case BOX:
            switch (geom1.type) {
                case SPHERE: return overlapSphereBox(pos1, geom1.sphere.radius, pos0, or0, geom0.box.halfExtents);
                case CAPSULE: return overlapCapsuleBox(pos1, or1, geom1.capsule.halfHeight, geom1.capsule.radius, pos0, or0, geom0.box.halfExtents);
                case BOX: return overlapBoxBox(pos0, or0, geom0.box.halfExtents, pos1, or1, geom1.box.halfExtents);
                case CONVEX_MESH: return overlapBoxConvexMesh(pos0, or0, geom0.box.halfExtents, pos1, or1, geom1.convex.mesh, geom1.convex.scale);
            }
        case CONVEX_MESH:
            switch (geom1.type) {
                case SPHERE: return overlapSphereConvexMesh(pos1, geom1.sphere.radius, pos0, or0, geom0.convex.mesh, geom0.convex.scale);
                case CAPSULE: return overlapCapsuleConvexMesh(pos1, or1, geom1.capsule.halfHeight, geom1.capsule.radius, pos0, or0, geom0.convex.mesh, geom0.convex.scale);
                case BOX: return overlapBoxConvexMesh(pos1, or1, geom1.box.halfExtents, pos0, or0, geom0.convex.mesh, geom0.convex.scale);
                case CONVEX_MESH: return overlapConvexMeshConvexMesh(pos0, or0, geom0.convex.mesh, geom0.convex.scale, pos1, or1, geom1.convex.mesh, geom1.convex.scale);
            }
    }
    return false;
}
