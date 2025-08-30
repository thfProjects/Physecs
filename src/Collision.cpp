#include "Collision.h"
#include <glm/gtx/quaternion.hpp>
#include "GeomUtil.h"
#include "Clipping.h"
#include "EPA.h"
#include "GJK.h"
#include "BoundsUtil.h"
#include "CollisionTriangleMesh.h"
#include "ContactsUtil.h"

using namespace physecs;

#if __cplusplus >= 202302L
    #define unreachable() std::unreachable()
#else
    #ifdef __GNUC__ // GCC, Clang, ICC
        #define unreachable() (__builtin_unreachable())
    #elifdef _MSC_VER // MSVC
        #define unreachable() (__assume(false))
    #else
        [[noreturn]] inline void unreachable_impl() {}
        #define unreachable() (unreachable_impl())
    #endif
#endif

namespace {
    struct BoxEdgeContactInfo {
        int edge0;
        int edge1;
    };

    struct BoxFaceContactInfo {
        int box;
        int axis;
    };

    enum BoxContactType { FACE, EDGE };

    thread_local std::vector<glm::vec2> polygon;
    thread_local std::vector<glm::vec2> clip;
}

static bool collisionSphereSphere(glm::vec3 pos0, float radius0, glm::vec3 pos1, float radius1, ContactManifold &result);
static bool collisionCapsuleCapsule(glm::vec3 pos0, glm::quat or0, float halfHeight0, float radius0, glm::vec3 pos1, glm::quat or1, float halfHeight1, float radius1, ContactManifold &result);
static bool collisionBoxBox(glm::vec3 pos0, glm::quat or0, glm::vec3 halfExtents0, glm::vec3 pos1, glm::quat or1, glm::vec3 halfExtents1, ContactManifold &result);
static bool collisionConvexMeshConvexMesh(glm::vec3 pos0, glm::quat or0, ConvexMesh* mesh0, glm::vec3 scale0, glm::vec3 pos1, glm::quat or1, ConvexMesh* mesh1, glm::vec3 scale1, ContactManifold &result);
static bool collisionSphereCapsule(glm::vec3 pos0, float radius0, glm::vec3 pos1, glm::quat or1, float halfHeight1, float radius1, ContactManifold &result);
static bool collisionSphereBox(glm::vec3 pos0, float radius0, glm::vec3 pos1, glm::quat or1, glm::vec3 halfExtents1, ContactManifold &result);
static bool collisionSphereConvexMesh(glm::vec3 pos0, float radius0, glm::vec3 pos1, glm::quat or1, ConvexMesh* mesh1, glm::vec3 scale1, ContactManifold &result);
static bool collisionCapsuleBox(glm::vec3 pos0, glm::quat or0, float halfHeight0, float radius0, glm::vec3 pos1, glm::quat or1, glm::vec3 halfExtents1, ContactManifold &result);
static bool collisionCapsuleConvexMesh(glm::vec3 pos0, glm::quat or0, float halfHeight0, float radius0, glm::vec3 pos1, glm::quat or1, ConvexMesh* mesh1, glm::vec3 scale1, ContactManifold &result);
static bool collisionBoxConvexMesh(glm::vec3 pos0, glm::quat or0, glm::vec3 halfExtents0, glm::vec3 pos1, glm::quat or1, ConvexMesh* mesh1, glm::vec3 scale1, ContactManifold &result);

static void generateBoxBoxFaceContacts(glm::mat3 uRef, glm::vec3 posRef, glm::vec3 halfExtentsRef, glm::mat3 uInc, glm::vec3 posInc, glm::vec3 halfExtentsInc, int referenceAxis, ContactPoint* contactPoints, int& numPoints) {

    glm::vec3 axis = uRef[referenceAxis];
    float refSign = glm::dot(axis, posInc - posRef) < 0 ? -1 : 1;

    //find incidence face
    float maxDot = 0;
    int incAxis;
    float incSign;
    for (int i = 0; i < 3; ++i) {
        float dot = glm::dot(-refSign * axis, uInc[i]);
        float absDot = glm::abs(dot);
        if (absDot > maxDot) {
            maxDot = absDot;
            incAxis = i;
            incSign = dot < 0 ? -1 : 1;
        }
    }

    polygon.reserve(8);
    polygon.resize(4);
    clip.resize(4);

    int incAxis1 = (incAxis + 1) % 3, incAxis2 = (incAxis + 2) % 3;

    //perform clip in local frame of reference box
    glm::mat3 inverseURef = glm::inverse(uRef);

    glm::vec3 incPlaneOrig = inverseURef * (posInc + uInc[incAxis] * halfExtentsInc[incAxis] * incSign - posRef);

    auto poly0 = incPlaneOrig + inverseURef * (uInc[incAxis1] * halfExtentsInc[incAxis1] + uInc[incAxis2] * halfExtentsInc[incAxis2]);
    auto poly1 = incPlaneOrig + inverseURef * (-uInc[incAxis1] * halfExtentsInc[incAxis1] + uInc[incAxis2] * halfExtentsInc[incAxis2]);
    auto poly2 = incPlaneOrig + inverseURef * (-uInc[incAxis1] * halfExtentsInc[incAxis1] - uInc[incAxis2] * halfExtentsInc[incAxis2]);
    auto poly3 = incPlaneOrig + inverseURef * (uInc[incAxis1] * halfExtentsInc[incAxis1] - uInc[incAxis2] * halfExtentsInc[incAxis2]);

    int clipX = (referenceAxis + 1) % 3;
    int clipY = (referenceAxis + 2) % 3;

    polygon[0] = {poly0[clipX], poly0[clipY]};
    polygon[1] = {poly1[clipX], poly1[clipY]};
    polygon[2] = {poly2[clipX], poly2[clipY]};
    polygon[3] = {poly3[clipX], poly3[clipY]};

    clip[0] = {halfExtentsRef[clipX], halfExtentsRef[clipY]};
    clip[1] = {halfExtentsRef[clipX], -halfExtentsRef[clipY]};
    clip[2] = {-halfExtentsRef[clipX], -halfExtentsRef[clipY]};
    clip[3] = {-halfExtentsRef[clipX], halfExtentsRef[clipY]};

    suthHodgClip(polygon, clip);

    //find polygon points which are penetrating
    glm::vec3 incPlaneNormal = inverseURef * uInc[incAxis];
    generateContactsPolygonBoxFace(posRef, uRef, referenceAxis, refSign, halfExtentsRef, incPlaneOrig, incPlaneNormal, polygon, clipX, clipY, contactPoints, numPoints);
}

static bool collisionSphereSphere(glm::vec3 pos0, float radius0, glm::vec3 pos1, float radius1, ContactManifold &result) {
    glm::vec3 d = pos1 - pos0;
    float radiusSum = radius0 + radius1;
    if (glm::dot(d, d) > radiusSum * radiusSum) return false;
    float dLen = glm::length(d);
    result.normal = dLen ? d / dLen : glm::vec3(0, 1, 0);
    result.numPoints = 1;
    result.points[0] = { pos0 + result.normal * radius0, pos1 - result.normal * radius1 };
    return true;
}

static bool collisionCapsuleCapsule(glm::vec3 pos0, glm::quat or0, float halfHeight0, float radius0, glm::vec3 pos1, glm::quat or1, float halfHeight1, float radius1, ContactManifold &result) {
    glm::vec3 up = glm::vec3(0, 1, 0);
    glm::vec3 v0 = or0 * up;
    glm::vec3 v1 = or1 * up;
    auto [point0, point1] = closestPointsBetweenSegments(pos0, v0, -halfHeight0, halfHeight0, pos1, v1, -halfHeight1, halfHeight1);
    glm::vec3 d = point1 - point0;
    float radiusSum = radius0 + radius1;
    if (glm::dot(d, d) > radiusSum * radiusSum) return false;
    float dLen = glm::length(d);
    result.normal = dLen? d / dLen : up;
    result.numPoints = 1;
    result.points[0] = { point0 + result.normal * radius0, point1 - result.normal * radius1 };
    return true;
}

static bool collisionBoxBox(glm::vec3 pos0, glm::quat or0, glm::vec3 halfExtents0, glm::vec3 pos1, glm::quat or1, glm::vec3 halfExtents1, ContactManifold &result) {
    float edgeOffset = 0.1f;
    const float edgeLimit = 0.999f;

    float ra, rb, l, d;

    BoxContactType contactType = FACE;
    union {
        BoxEdgeContactInfo edgeInfo;
        BoxFaceContactInfo faceInfo;
    };

    float max = std::numeric_limits<float>::lowest();

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
        if (d > max) {
            max = d;
            contactType = FACE;
            faceInfo = { 0, i };
        }
    }

    // Test axes L = B0, L = B1, L = B2
    for (int i = 0; i < 3; i++) {
        ra = halfExtents0[0] * absR[0][i] + halfExtents0[1] * absR[1][i] + halfExtents0[2] * absR[2][i];
        rb = halfExtents1[i];
        l = glm::abs(t[0] * r[0][i] + t[1] * r[1][i] + t[2] * r[2][i]);
        d = l - ra - rb;
        if (d > 0) return false;
        if (d > max) {
            max = d;
            contactType = FACE;
            faceInfo = { 1, i };
        }
    }

    // Test axis L = A0 x B0
    ra = halfExtents0[1] * absR[2][0] + halfExtents0[2] * absR[1][0];
    rb = halfExtents1[1] * absR[0][2] + halfExtents1[2] * absR[0][1];
    l = glm::abs(t[2] * r[1][0] - t[1] * r[2][0]);
    d = l - ra - rb;
    if (d > 0) return false;
    if (absR[0][0] < edgeLimit && d > max + edgeOffset) {
        max = d;
        contactType = EDGE;
        edgeInfo = { 0, 0 };
        edgeOffset = 0;
    }

    // Test axis L = A0 x B1
    ra = halfExtents0[1] * absR[2][1] + halfExtents0[2] * absR[1][1];
    rb = halfExtents1[0] * absR[0][2] + halfExtents1[2] * absR[0][0];
    l = glm::abs(t[2] * r[1][1] - t[1] * r[2][1]);
    d = l - ra - rb;
    if (d > 0) return false;
    if (absR[0][1] < edgeLimit && d > max + edgeOffset) {
        max = d;
        contactType = EDGE;
        edgeInfo = { 0, 1 };
        edgeOffset = 0;
    }

    // Test axis L = A0 x B2
    ra = halfExtents0[1] * absR[2][2] + halfExtents0[2] * absR[1][2];
    rb = halfExtents1[0] * absR[0][1] + halfExtents1[1] * absR[0][0];
    l = glm::abs(t[2] * r[1][2] - t[1] * r[2][2]);
    d = l - ra - rb;
    if (d > 0) return false;
    if (absR[0][2] < edgeLimit && d > max + edgeOffset) {
        max = d;
        contactType = EDGE;
        edgeInfo = { 0, 2 };
        edgeOffset = 0;
    }

    // Test axis L = A1 x B0
    ra = halfExtents0[0] * absR[2][0] + halfExtents0[2] * absR[0][0];
    rb = halfExtents1[1] * absR[1][2] + halfExtents1[2] * absR[1][1];
    l = glm::abs(t[0] * r[2][0] - t[2] * r[0][0]);
    d = l - ra - rb;
    if (d > 0) return false;
    if (absR[1][0] < edgeLimit && d > max + edgeOffset) {
        max = d;
        contactType = EDGE;
        edgeInfo = { 1, 0 };
        edgeOffset = 0;
    }

    // Test axis L = A1 x B1
    ra = halfExtents0[0] * absR[2][1] + halfExtents0[2] * absR[0][1];
    rb = halfExtents1[0] * absR[1][2] + halfExtents1[2] * absR[1][0];
    l = glm::abs(t[0] * r[2][1] - t[2] * r[0][1]);
    d = l - ra - rb;
    if (d > 0) return false;
    if (absR[1][1] < edgeLimit && d > max + edgeOffset) {
        max = d;
        contactType = EDGE;
        edgeInfo = { 1, 1 };
        edgeOffset = 0;
    }

    // Test axis L = A1 x B2
    ra = halfExtents0[0] * absR[2][2] + halfExtents0[2] * absR[0][2];
    rb = halfExtents1[0] * absR[1][1] + halfExtents1[1] * absR[1][0];
    l = glm::abs(t[0] * r[2][2] - t[2] * r[0][2]);
    d = l - ra - rb;
    if (d > 0) return false;
    if (absR[1][2] < edgeLimit && d > max + edgeOffset) {
        max = d;
        contactType = EDGE;
        edgeInfo = { 1, 2 };
        edgeOffset = 0;
    }

    // Test axis L = A2 x B0
    ra = halfExtents0[0] * absR[1][0] + halfExtents0[1] * absR[0][0];
    rb = halfExtents1[1] * absR[2][2] + halfExtents1[2] * absR[2][1];
    l = glm::abs(t[1] * r[0][0] - t[0] * r[1][0]);
    d = l - ra - rb;
    if (d > 0) return false;
    if (absR[2][0] < edgeLimit && d > max + edgeOffset) {
        max = d;
        contactType = EDGE;
        edgeInfo = { 2, 0 };
        edgeOffset = 0;
    }

    // Test axis L = A2 x B1
    ra = halfExtents0[0] * absR[1][1] + halfExtents0[1] * absR[0][1];
    rb = halfExtents1[0] * absR[2][2] + halfExtents1[2] * absR[2][0];
    l = glm::abs(t[1] * r[0][1] - t[0] * r[1][1]);
    d = l - ra - rb;
    if (d > 0) return false;
    if (absR[2][1] < edgeLimit && d > max + edgeOffset) {
        max = d;
        contactType = EDGE;
        edgeInfo = { 2, 1 };
        edgeOffset = 0;
    }

    // Test axis L = A2 x B2
    ra = halfExtents0[0] * absR[1][2] + halfExtents0[1] * absR[0][2];
    rb = halfExtents1[0] * absR[2][1] + halfExtents1[1] * absR[2][0];
    l = glm::abs(t[1] * r[0][2] - t[0] * r[1][2]);
    d = l - ra - rb;
    if (d > 0) return false;
    if (absR[2][2] < edgeLimit && d > max + edgeOffset) {
        max = d;
        contactType = EDGE;
        edgeInfo = { 2, 2 };
    }

    switch (contactType) {
        case FACE: {
            glm::vec3 axis;
            ContactPoint contactPoints[4];
            int numPoints;
            if (faceInfo.box) {
                axis = u1[faceInfo.axis];
                generateBoxBoxFaceContacts(u1, pos1, halfExtents1, u0, pos0, halfExtents0, faceInfo.axis, contactPoints, numPoints);
                for (int i = 0; i < numPoints; i++) {
                    result.points[i] = { contactPoints[i].position1, contactPoints[i].position0 };
                }
            } else {
                axis = u0[faceInfo.axis];
                generateBoxBoxFaceContacts(u0, pos0, halfExtents0, u1, pos1, halfExtents1, faceInfo.axis, contactPoints, numPoints);
                for (int i = 0; i < numPoints; i++) {
                    result.points[i] = contactPoints[i];
                }
            }
            result.numPoints = numPoints;
            result.normal = glm::dot(axis, pos1 - pos0) < 0 ? -axis : axis;
        }
        break;
        case EDGE: {
            glm::vec3 axis = glm::cross(u0[edgeInfo.edge0], u1[edgeInfo.edge1]);
            result.normal = glm::normalize(glm::dot(axis, pos1 - pos0) < 0 ? -axis : axis);
            glm::vec3 sign0(-1);
            sign0[(edgeInfo.edge0 + 1) % 3] = glm::dot(u0[(edgeInfo.edge0 + 1) % 3], result.normal) < 0 ? -1 : 1;
            sign0[(edgeInfo.edge0 + 2) % 3] = glm::dot(u0[(edgeInfo.edge0 + 2) % 3], result.normal) < 0 ? -1 : 1;
            glm::vec3 p0 = pos0 + sign0[0] * halfExtents0[0] * u0[0] + sign0[1] * halfExtents0[1] * u0[1] + sign0[2] * halfExtents0[2] * u0[2];
            glm::vec3 sign1(-1);
            sign1[(edgeInfo.edge1 + 1) % 3] = glm::dot(u1[(edgeInfo.edge1 + 1) % 3], result.normal) > 0 ? -1 : 1;
            sign1[(edgeInfo.edge1 + 2) % 3] = glm::dot(u1[(edgeInfo.edge1 + 2) % 3], result.normal) > 0 ? -1 : 1;
            glm::vec3 p1 = pos1 + sign1[0] * halfExtents1[0] * u1[0] + sign1[1] * halfExtents1[1] * u1[1] + sign1[2] * halfExtents1[2] * u1[2];
            result.numPoints = 1;
            auto [point0, point1] = closestPointsBetweenSegments(p0, u0[edgeInfo.edge0], 0, halfExtents0[edgeInfo.edge0] * 2, p1, u1[edgeInfo.edge1], 0, halfExtents1[edgeInfo.edge1] * 2);
            result.points[0] = { point0, point1 };
        }
    }

    return true;
}

static void generateConvexMeshConvexMeshContacts(glm::vec3 pos0, glm::quat or0, ConvexMesh *mesh0, glm::vec3 scale0, glm::vec3 pos1, glm::quat or1, ConvexMesh *mesh1, glm::vec3 scale1, glm::vec3 normal, ContactPoint* contactPoints, int& numPoints) {
    int convex0FaceIndex = 0;
    float maxDot = 0;
    for (int i = 0; i < mesh0->faces.size(); ++i) {
        float dot = glm::dot(normal, glm::normalize(or0 * (mesh0->faces[i].normal / scale0)));
        if (dot > maxDot) {
            maxDot = dot;
            convex0FaceIndex = i;
        }
    }
    auto& convex0Face = mesh0->faces[convex0FaceIndex];

    int convex1FaceIndex = 0;
    float minDot = 0;
    for (int i = 0; i < mesh1->faces.size(); ++i) {
        float dot = glm::dot(normal, glm::normalize(or1 * (mesh1->faces[i].normal / scale1)));
        if (dot < minDot) {
            minDot = dot;
            convex1FaceIndex = i;
        }
    }
    auto& convex1Face = mesh1->faces[convex1FaceIndex];

    glm::mat3 convex0ToWorld = glm::toMat3(or0);
    glm::mat3 worldToConvex0 = glm::transpose(convex0ToWorld);

    glm::vec3 refPlaneOrigin = scale0 * convex0Face.centroid;

    glm::vec3 u0 = glm::normalize(scale0 * mesh0->vertices[convex0Face.indices[0]] - refPlaneOrigin);
    glm::vec3 u1 = glm::normalize(convex0Face.normal / scale0);
    glm::vec3 u2 = glm::cross(u0, u1);

    glm::mat3 convex0ToRef = glm::transpose(glm::mat3(u0, u1, u2));
    glm::mat3 worldToRef = convex0ToRef * worldToConvex0;;

    int clipX = 2;
    int clipY = 0;

    clip.resize(convex0Face.indices.size());
    for (int i = 0; i < convex0Face.indices.size(); ++i) {
        int index = convex0Face.indices[i];
        auto vertex = convex0ToRef * (scale0 * mesh0->vertices[index]);
        clip[i] = { vertex[clipX], vertex[clipY] };
    }

    polygon.resize(convex1Face.indices.size());
    for (int i = 0; i < convex1Face.indices.size(); ++i) {
        int index = convex1Face.indices[i];
        auto vertex = worldToRef * (pos1 + or1 * (scale1 * mesh1->vertices[index]) - pos0);
        polygon[i] = { vertex[clipX], vertex[clipY] };
    }

    suthHodgClip(polygon, clip);

    glm::vec3 incPlaneOrigin = worldToRef * (pos1 + or1 * (scale1 * convex1Face.centroid) - pos0);
    glm::vec3 incPlaneNormal = worldToRef * (or1 * glm::normalize(convex1Face.normal / scale1));

    glm::mat3 refToWorld = glm::transpose(worldToRef);

    generateContactsPolygonPolygonFace(pos0, refToWorld, refPlaneOrigin, u1, incPlaneOrigin, incPlaneNormal, polygon, clipX, clipY, contactPoints, numPoints);
}

static bool collisionConvexMeshConvexMesh(glm::vec3 pos0, glm::quat or0, ConvexMesh *mesh0, glm::vec3 scale0, glm::vec3 pos1, glm::quat or1, ConvexMesh *mesh1, glm::vec3 scale1, ContactManifold &result) {
    ConvexMeshSupportFunction convexMesh0Support{ pos0, glm::toMat3(or0), mesh0->vertices, scale0 };
    ConvexMeshSupportFunction convexMesh1Support{ pos1, glm::toMat3(or1), mesh1->vertices, scale1 };

    GjkSimplex s;
    if(!GJK<ConvexMeshSupportFunction, ConvexMeshSupportFunction>::gjk(convexMesh0Support, convexMesh1Support, pos1 - pos0, s)) return false;

    result.normal = EPA<ConvexMeshSupportFunction, ConvexMeshSupportFunction>::epa(convexMesh0Support, convexMesh1Support, s, result.points[0].position0, result.points[0].position1);
    generateConvexMeshConvexMeshContacts(pos0, or0, mesh0, scale0, pos1, or1, mesh1, scale1, result.normal, result.points, result.numPoints);
    if (!result.numPoints) result.numPoints = 1;

    return true;
}

static bool collisionSphereCapsule(glm::vec3 pos0, float radius0, glm::vec3 pos1, glm::quat or1, float halfHeight1, float radius1, ContactManifold &result) {
    glm::vec3 up = glm::vec3(0, 1, 0);
    glm::vec3 dir = or1 * up;
    glm::vec3 q = closestPointOnSegment(pos0, pos1, dir, -halfHeight1, halfHeight1);
    glm::vec3 v = q - pos0;
    float radiusSum = radius0 + radius1;
    if (glm::dot(v, v) > radiusSum * radiusSum) return false;
    float vLen = glm::length(v);
    result.normal = vLen ? v / vLen : up;
    result.numPoints = 1;
    result.points[0] = { pos0 + result.normal * radius0, q - result.normal * radius1 };
    return true;
}

static bool collisionSphereBox(glm::vec3 pos0, float radius0, glm::vec3 pos1, glm::quat or1, glm::vec3 halfExtents1, ContactManifold &result) {
    //basis vectors
    glm::mat3 u1 = glm::toMat3(or1);

    glm::vec3 d = pos0 - pos1;
    d = glm::vec3(glm::dot(d, u1[0]), glm::dot(d, u1[1]), glm::dot(d, u1[2]));

    //closest point on OBB to sphere
    glm::vec3 q = pos1 + u1 * glm::clamp(d, -halfExtents1, halfExtents1);

    glm::vec3 v = q - pos0;

    if (glm::dot(v, v) > radius0 * radius0) return false;

    float vLen = glm::length(v);

    if (vLen) {
        result.normal = v / vLen;
        result.numPoints = 1;
        result.points[0] = { pos0 + result.normal * radius0, q };
    }
    else {
        // penetration based separation
        float min = std::numeric_limits<float>::max();
        int separationAxis = 0;
        for (int i = 0; i < 3; i++) {
            float depth = halfExtents1[i] - glm::abs(d[i]);
            if (depth < min) {
                min = depth;
                separationAxis = i;
            }
        }

        float sign = d[separationAxis] ? glm::sign(d[separationAxis]) : 1.f;

        glm::vec3 boxParam = d;
        boxParam[separationAxis] = sign * halfExtents1[separationAxis];

        result.normal = -sign * u1[separationAxis];
        result.numPoints = 1;
        result.points[0] = { pos0, pos1 + u1 * boxParam };
    }

    return true;
}

static bool collisionSphereConvexMesh(glm::vec3 pos0, float radius0, glm::vec3 pos1, glm::quat or1, ConvexMesh *mesh1, glm::vec3 scale1, ContactManifold &result) {
    SphereSupportFunction sphereSupport{ pos0, radius0 };
    ConvexMeshSupportFunction convexMeshSupport{ pos1, glm::toMat3(or1), mesh1->vertices, scale1 };

    GjkSimplex s;
    if (!GJK<SphereSupportFunction, ConvexMeshSupportFunction>::gjk(sphereSupport, convexMeshSupport, pos1 - pos0, s)) return false;

    result.normal = EPA<SphereSupportFunction, ConvexMeshSupportFunction>::epa(sphereSupport, convexMeshSupport, s, result.points[0].position0, result.points[0].position1);
    result.numPoints = 1;

    return true;
}

static void generateCapsuleBoxContacts(glm::vec3 p0Local, glm::vec3 p1Local, float radius, glm::vec3 normal, glm::vec3 boxCenter, glm::vec3 halfExtents, glm::mat3 boxBasis, ContactManifold& result) {
    int boxAxis = 0;
    float boxAxisSign = 0;
    float maxDot = 0;
    for (int i = 0; i < 3; ++i) {
        float dot = glm::dot(normal, boxBasis[i]);
        float absDot = glm::abs(dot);
        if (absDot > maxDot) {
            maxDot = absDot;
            boxAxis = i;
            boxAxisSign = dot < 0 ? 1 : -1;
        }
    }

    int clipX = (boxAxis + 1) % 3;
    int clipY = (boxAxis + 2) % 3;

    glm::vec2 line0 = glm::vec2(p0Local[clipX], p0Local[clipY]);
    glm::vec2 line1 = glm::vec2(p1Local[clipX], p1Local[clipY]);

    glm::vec2 clip[4];

    clip[0] = {halfExtents[clipX], halfExtents[clipY]};
    clip[1] = {halfExtents[clipX], -halfExtents[clipY]};
    clip[2] = {-halfExtents[clipX], -halfExtents[clipY]};
    clip[3] = {-halfExtents[clipX], halfExtents[clipY]};

    if (clipLine(line0, line1, clip, 4)) {

        float onBoxAxis = boxAxisSign * halfExtents[boxAxis];

        glm::vec3 p0, p1;

        float distX = p0Local[clipX] - p1Local[clipX];
        float distY = p0Local[clipY] - p1Local[clipY];
        float absDistX = glm::abs(distX);
        float absDistY = glm::abs(distY);

        if (absDistX > absDistY) {
            p0[boxAxis] = glm::mix(p0Local[boxAxis], p1Local[boxAxis], (p0Local[clipX] - line0.x) / distX);
            p1[boxAxis] = glm::mix(p1Local[boxAxis], p0Local[boxAxis], (p1Local[clipX] - line1.x) / -distX);
        }
        else if (absDistY > absDistX) {
            p0[boxAxis] = glm::mix(p0Local[boxAxis], p1Local[boxAxis], (p0Local[clipY] - line0.y) / distY);
            p1[boxAxis] = glm::mix(p1Local[boxAxis], p0Local[boxAxis], (p1Local[clipY] - line1.y) / -distY);
        }
        else {
            p0[boxAxis] = p0Local[boxAxis];
            p1[boxAxis] = p1Local[boxAxis];
        }

        p0[boxAxis] -= boxAxisSign * radius;
        p1[boxAxis] -= boxAxisSign * radius;

        int numPoints = 0;

        if (glm::abs(p0[boxAxis]) < halfExtents[boxAxis]) {
            p0[clipX] = line0.x;
            p0[clipY] = line0.y;

            glm::vec3 onBox0 = p0;
            onBox0[boxAxis] = onBoxAxis;

            result.points[numPoints++] = { boxCenter + boxBasis * p0, boxCenter + boxBasis * onBox0 };
        }

        if (glm::abs(p1[boxAxis]) < halfExtents[boxAxis]) {
            p1[clipX] = line1.x;
            p1[clipY] = line1.y;

            glm::vec3 onBox1 = p1;
            onBox1[boxAxis] = onBoxAxis;

            result.points[numPoints++] = { boxCenter + boxBasis * p1, boxCenter + boxBasis * onBox1 };
        }

        result.numPoints = numPoints;
    }
    else {
        result.numPoints = 0;
    }
}

static bool collisionCapsuleBox(glm::vec3 pos0, glm::quat or0, float halfHeight0, float radius0, glm::vec3 pos1, glm::quat or1, glm::vec3 halfExtents1, ContactManifold &result) {
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

    glm::vec3 onSegment = pos0 + dir * t;
    glm::vec3 onBox = pos1 + u1 * q;

    glm::vec3 normal = onBox - onSegment;

    float nLen = glm::length(normal);

    if (nLen) {
        result.normal = normal / nLen;
    }
    else {
        //find minimum separation axis
        glm::vec3 r;
        glm::vec3 absR;

        for (int i = 0; i < 3; ++i) {
            r[i] = glm::dot(dir, u1[i]);
            absR[i] = glm::abs(r[i]);
        }

        float max = std::numeric_limits<float>::lowest();
        BoxContactType contactType = FACE;
        int axis;

        float ra, rb, l, d;
        for (int i = 0; i < 3; i++) {
            ra = halfHeight0 * absR[i] + radius0;
            rb = halfExtents1[i];
            l = glm::abs(p[i]);
            d = l - ra - rb;
            if (d > max) {
                max = d;
                contactType = FACE;
                axis = i;
            }
        }

        const float edgeLimit = 0.999f;
        float edgeOffset = 0.1f;

        // Test axis L = dir x B0
        rb = halfExtents1[1] * absR[2] + halfExtents1[2] * absR[1];
        l = glm::abs(p[2] * r[1] - p[1] * r[2]);
        d = l - radius0 - rb;
        if (absR[0] < edgeLimit && d > max + edgeOffset) {
            max = d;
            contactType = EDGE;
            axis = 0;
            edgeOffset = 0;
        }

        // Test axis L = dir x B1
        rb = halfExtents1[0] * absR[2] + halfExtents1[2] * absR[0];
        l = glm::abs(p[0] * r[2] - p[2] * r[0]);
        d = l - radius0 - rb;
        if (absR[1] < edgeLimit && d > max + edgeOffset) {
            max = d;
            contactType = EDGE;
            axis = 1;
            edgeOffset = 0;
        }

        // Test axis L = dir x B2
        rb = halfExtents1[0] * absR[1] + halfExtents1[1] * absR[0];
        l = glm::abs(p[1] * r[0] - p[0] * r[1]);
        d = l - radius0 - rb;
        if (absR[2] < edgeLimit && d > max + edgeOffset) {
            max = d;
            contactType = EDGE;
            axis = 2;
        }

        glm::vec3 separationAxis;
        switch (contactType) {
            case FACE:
                separationAxis = u1[axis];
                break;
            case EDGE:
                separationAxis = glm::cross(dir, u1[axis]);
        }
        result.normal = glm::normalize(glm::dot(separationAxis, pos1 - pos0) > 0 ? separationAxis : -separationAxis);
    }

    generateCapsuleBoxContacts(p - dirLc * halfHeight0, p + dirLc * halfHeight0, radius0, result.normal, pos1, halfExtents1, u1, result);

    if (!result.numPoints) {
        result.numPoints = 1;
        result.points[0] = { onSegment + result.normal * radius0, onBox };
    }

    return true;
}

static void generateCapsuleConvexMeshContacts(glm::vec3 p0Local, glm::vec3 p1Local, float radius, glm::vec3 meshPos, const glm::mat3 &convexToWorld, ConvexMesh* mesh, glm::vec3 meshScale, ContactManifold& result) {
    int convexFaceIndex = 0;
    float minDot = 0;
    for (int i = 0; i < mesh->faces.size(); ++i) {
        float dot = glm::dot(result.normal, glm::normalize(convexToWorld * (mesh->faces[i].normal / meshScale)));
        if (dot < minDot) {
            minDot = dot;
            convexFaceIndex = i;
        }
    }
    auto& convexFace = mesh->faces[convexFaceIndex];

    glm::vec3 refPlaneOrigin = meshScale * convexFace.centroid;

    glm::vec3 u0 = glm::normalize(meshScale * mesh->vertices[convexFace.indices[0]] - refPlaneOrigin);
    glm::vec3 u1 = glm::normalize(convexFace.normal / meshScale);
    glm::vec3 u2 = glm::cross(u0, u1);

    glm::mat3 convexToRef = glm::transpose(glm::mat3(u0, u1, u2));

    const int faceAxis = 1;
    const int clipX = 2;
    const int clipY = 0;

    clip.resize(convexFace.indices.size());
    for (int i = 0; i < convexFace.indices.size(); ++i) {
        int index = convexFace.indices[i];
        auto vertex = convexToRef * (meshScale * mesh->vertices[index]);
        clip[i] = { vertex[clipX], vertex[clipY] };
    }

    glm::vec3 p0Ref = convexToRef * p0Local;
    glm::vec3 p1Ref = convexToRef * p1Local;

    glm::vec2 line0 = { p0Ref[clipX], p0Ref[clipY] };
    glm::vec2 line1 = { p1Ref[clipX], p1Ref[clipY] };

    if (clipLine(line0, line1, clip.data(), clip.size())) {
        glm::mat3 refToWorld = convexToWorld * glm::transpose(convexToRef);
        float distToPlane = glm::dot(refPlaneOrigin, u1);

        glm::vec3 p0, p1;

        float distX = p0Ref[clipX] - p1Ref[clipX];
        float distY = p0Ref[clipY] - p1Ref[clipY];
        float absDistX = glm::abs(distX);
        float absDistY = glm::abs(distY);

        if (absDistX && absDistX >= absDistY) {
            p0[faceAxis] = glm::mix(p0Ref[faceAxis], p1Ref[faceAxis], (p0Ref[clipX] - line0.x) / distX);
            p1[faceAxis] = glm::mix(p1Ref[faceAxis], p0Ref[faceAxis], (p1Ref[clipX] - line1.x) / -distX);
        }
        else if (absDistY > absDistX) {
            p0[faceAxis] = glm::mix(p0Ref[faceAxis], p1Ref[faceAxis], (p0Ref[clipY] - line0.y) / distY);
            p1[faceAxis] = glm::mix(p1Ref[faceAxis], p0Ref[faceAxis], (p1Ref[clipY] - line1.y) / -distY);
        }
        else {
            p0[faceAxis] = p0Ref[faceAxis];
            p1[faceAxis] = p1Ref[faceAxis];
        }

        p0[faceAxis] -= radius;
        p1[faceAxis] -= radius;

        int numPoints = 0;

        if (p0[faceAxis] < distToPlane) {
            p0[clipX] = line0.x;
            p0[clipY] = line0.y;

            glm::vec3 onFace = p0;
            onFace[faceAxis] = distToPlane;

            result.points[numPoints++] = { meshPos + refToWorld * p0, meshPos + refToWorld * onFace };
        }

        if (p1[faceAxis] < distToPlane) {
            p1[clipX] = line1.x;
            p1[clipY] = line1.y;

            glm::vec3 onFace = p1;
            onFace[faceAxis] = distToPlane;

            result.points[numPoints++] = { meshPos + refToWorld * p1, meshPos + refToWorld * onFace };
        }

        result.numPoints = numPoints;
    }
    else {
        result.numPoints = 0;
    }
}

static bool collisionCapsuleConvexMesh(glm::vec3 pos0, glm::quat or0, float halfHeight0, float radius0, glm::vec3 pos1, glm::quat or1, ConvexMesh *mesh1, glm::vec3 scale1, ContactManifold &result) {
    glm::mat3 convexToWorld = glm::toMat3(or1);
    glm::vec3 capsuleAxis = or0 * glm::vec3(0, 1, 0);

    CapsuleSupportFunction capsuleSupport{ pos0, capsuleAxis, halfHeight0, radius0 };
    ConvexMeshSupportFunction convexMeshSupport{ pos1, convexToWorld, mesh1->vertices, scale1 };

    GjkSimplex s;
    if(!GJK<CapsuleSupportFunction, ConvexMeshSupportFunction>::gjk(capsuleSupport, convexMeshSupport, pos1 - pos0, s)) return false;

    result.normal = EPA<CapsuleSupportFunction, ConvexMeshSupportFunction>::epa(capsuleSupport, convexMeshSupport, s, result.points[0].position0, result.points[0].position1);

    glm::mat3 worldToConvex = glm::inverse(convexToWorld);
    glm::vec3 p = worldToConvex * (pos0 - pos1);
    glm::vec3 axisLc = worldToConvex * capsuleAxis;
    generateCapsuleConvexMeshContacts(p + axisLc * halfHeight0, p - axisLc * halfHeight0, radius0, pos1, convexToWorld, mesh1, scale1, result);

    if (!result.numPoints) result.numPoints = 1;

    return true;
}

static void generateBoxConvexMeshContacts(
    glm::vec3 boxCenter,
    glm::mat3 boxBasis,
    glm::vec3 halfExtents,
    glm::vec3 convexPos,
    glm::quat convexOr,
    ConvexMesh* convexMesh,
    glm::vec3 convexScale,
    glm::vec3 normal,
    ContactPoint* contactPoints,
    int& numPoints)
{
    int boxAxis = 0;
    float boxAxisSign = 0;
    float maxDot = 0;
    for (int i = 0; i < 3; ++i) {
        float dot = glm::dot(normal, boxBasis[i]);
        float absDot = glm::abs(dot);
        if (absDot > maxDot) {
            maxDot = absDot;
            boxAxis = i;
            boxAxisSign = dot < 0 ? -1 : 1;
        }
    }

    int convexFaceIndex = 0;
    float minDot = 0;
    for (int i = 0; i < convexMesh->faces.size(); ++i) {
        float dot = glm::dot(normal, glm::normalize(convexOr * (convexMesh->faces[i].normal / convexScale)));
        if (dot < minDot) {
            minDot = dot;
            convexFaceIndex = i;
        }
    }
    auto& convexFace = convexMesh->faces[convexFaceIndex];

    int clipX = (boxAxis + 1) % 3;
    int clipY = (boxAxis + 2) % 3;

    glm::mat3 worldToBox = glm::transpose(boxBasis);

    polygon.reserve(convexFace.indices.size() + 4);
    polygon.resize(convexFace.indices.size());

    for (int i = 0; i < convexFace.indices.size(); ++i) {
        int index = convexFace.indices[i];
        glm::vec3 vertex = worldToBox * (convexPos + convexOr * (convexScale * convexMesh->vertices[index]) - boxCenter);
        polygon[i] = { vertex[clipX], vertex[clipY] };
    }

    clip.resize(4);

    clip[0] = {halfExtents[clipX], halfExtents[clipY]};
    clip[1] = {halfExtents[clipX], -halfExtents[clipY]};
    clip[2] = {-halfExtents[clipX], -halfExtents[clipY]};
    clip[3] = {-halfExtents[clipX], halfExtents[clipY]};

    suthHodgClip(polygon, clip);

    glm::vec3 incPlaneOrig = worldToBox * (convexPos + convexOr * (convexScale * convexFace.centroid) - boxCenter);
    glm::vec3 incPlaneNormal = glm::normalize(worldToBox * (convexOr * (convexFace.normal / convexScale)));
    generateContactsPolygonBoxFace(boxCenter, boxBasis, boxAxis, boxAxisSign, halfExtents, incPlaneOrig, incPlaneNormal, polygon, clipX, clipY, contactPoints, numPoints);
}

static bool collisionBoxConvexMesh(glm::vec3 pos0, glm::quat or0, glm::vec3 halfExtents0, glm::vec3 pos1, glm::quat or1, ConvexMesh *mesh1, glm::vec3 scale1, ContactManifold &result) {
    glm::mat3 boxBasis = glm::toMat3(or0);

    BoxSupportFunction boxSupport{ pos0, boxBasis, halfExtents0 };
    ConvexMeshSupportFunction convexMeshSupport{ pos1, glm::toMat3(or1), mesh1->vertices, scale1 };

    GjkSimplex s;
    if(!GJK<BoxSupportFunction, ConvexMeshSupportFunction>::gjk(boxSupport, convexMeshSupport, pos1 - pos0, s)) return false;

    result.normal = EPA<BoxSupportFunction, ConvexMeshSupportFunction>::epa(boxSupport, convexMeshSupport, s, result.points[0].position0, result.points[0].position1);
    generateBoxConvexMeshContacts(pos0, boxBasis, halfExtents0, pos1, or1, mesh1, scale1, result.normal, result.points, result.numPoints);
    if (!result.numPoints) result.numPoints = 1;

    return true;
}

static void flipContacts(ContactManifold &result) {
    result.normal = -result.normal;
    for (int i = 0; i < result.numPoints; ++i)
        result.points[i] = { result.points[i].position1, result.points[i].position0 };
}

bool physecs::collision(glm::vec3 pos0, glm::quat or0, Geometry &geom0, glm::vec3 pos1, glm::quat or1, Geometry &geom1, std::vector<ContactManifold> &results) {
    PhysecsZoneScoped;
    if (geom0.type == TRIANGLE_MESH) {
        auto triangle0 = geom0.triangleMesh;
        bool isCollision = collisionTriangleMesh(pos1, or1, geom1, pos0, or0, triangle0.mesh, results);
        for (auto& result : results) flipContacts(result);
        return isCollision;
    }

    if (geom1.type == TRIANGLE_MESH) {
        auto triangle1 = geom1.triangleMesh;
        return collisionTriangleMesh(pos0, or0, geom0, pos1, or1, triangle1.mesh, results);
    }

    results.push_back(ContactManifold());
    switch (geom0.type) {
        case SPHERE: {
            auto sphere0 = geom0.sphere;
            switch (geom1.type) {
                case SPHERE: {
                    auto sphere1 = geom1.sphere;
                    return collisionSphereSphere(pos0, sphere0.radius, pos1, sphere1.radius, results[0]);
                }
                case CAPSULE: {
                    auto capsule1 = geom1.capsule;
                    return collisionSphereCapsule(pos0, sphere0.radius, pos1, or1, capsule1.halfHeight, capsule1.radius, results[0]);
                }
                case BOX: {
                    auto box1 = geom1.box;
                    return collisionSphereBox(pos0, sphere0.radius, pos1, or1, box1.halfExtents, results[0]);
                }
                case CONVEX_MESH: {
                    auto convex1 = geom1.convex;
                    return collisionSphereConvexMesh(pos0, sphere0.radius, pos1, or1, convex1.mesh, convex1.scale, results[0]);
                }
                default:
                    unreachable();
            }
        }
        case CAPSULE: {
            auto capsule0 = geom0.capsule;
            switch (geom1.type) {
                case SPHERE: {
                    auto sphere1 = geom1.sphere;
                    bool isCollision = collisionSphereCapsule(pos1, sphere1.radius, pos0, or0, capsule0.halfHeight, capsule0.radius, results[0]);
                    flipContacts(results[0]);
                    return isCollision;
                }
                case CAPSULE: {
                    auto capsule1 = geom1.capsule;
                    return collisionCapsuleCapsule(pos0, or0, capsule0.halfHeight, capsule0.radius, pos1, or1, capsule1.halfHeight, capsule1.radius, results[0]);
                }
                case BOX: {
                    auto box1 = geom1.box;
                    return collisionCapsuleBox(pos0, or0, capsule0.halfHeight, capsule0.radius, pos1, or1, box1.halfExtents, results[0]);
                }
                case CONVEX_MESH: {
                    auto convex1 = geom1.convex;
                    return collisionCapsuleConvexMesh(pos0, or0, capsule0.halfHeight, capsule0.radius, pos1, or1, convex1.mesh, convex1.scale, results[0]);
                }
                default:
                    unreachable();
            }
        }
        case BOX: {
            auto box0 = geom0.box;
            switch (geom1.type) {
                case SPHERE: {
                    auto sphere1 = geom1.sphere;
                    bool isCollision = collisionSphereBox(pos1, sphere1.radius, pos0, or0, box0.halfExtents, results[0]);
                    flipContacts(results[0]);
                    return isCollision;
                }
                case CAPSULE: {
                    auto capsule1 = geom1.capsule;
                    bool isCollision = collisionCapsuleBox(pos1, or1, capsule1.halfHeight, capsule1.radius, pos0, or0, box0.halfExtents, results[0]);
                    flipContacts(results[0]);
                    return isCollision;
                }
                case BOX: {
                    auto box1 = geom1.box;
                    return collisionBoxBox(pos0, or0, box0.halfExtents, pos1, or1, box1.halfExtents, results[0]);
                }
                case CONVEX_MESH: {
                    auto convex1 = geom1.convex;
                    return collisionBoxConvexMesh(pos0, or0, box0.halfExtents, pos1, or1, convex1.mesh, convex1.scale, results[0]);
                }
                default:
                    unreachable();
            }
        }
        case CONVEX_MESH: {
            auto convex0 = geom0.convex;
            switch (geom1.type) {
                case SPHERE: {
                    auto sphere1 = geom1.sphere;
                    bool isCollision = collisionSphereConvexMesh(pos1, sphere1.radius, pos0, or0, convex0.mesh, convex0.scale, results[0]);
                    flipContacts(results[0]);
                    return isCollision;
                }
                case CAPSULE: {
                    auto capsule1 = geom1.capsule;
                    bool isCollision = collisionCapsuleConvexMesh(pos1, or1, capsule1.halfHeight, capsule1.radius, pos0, or0, convex0.mesh, convex0.scale, results[0]);
                    flipContacts(results[0]);
                    return isCollision;
                }
                case BOX: {
                    auto box1 = geom1.box;
                    bool isCollision = collisionBoxConvexMesh(pos1, or1, box1.halfExtents, pos0, or0, convex0.mesh, convex0.scale, results[0]);
                    flipContacts(results[0]);
                    return isCollision;
                }
                case CONVEX_MESH: {
                    auto convex1 = geom1.convex;
                    return collisionConvexMeshConvexMesh(pos0, or0, convex0.mesh, convex0.scale, pos1, or1, convex1.mesh, convex1.scale, results[0]);
                }
                default: unreachable();
            }
        }
        default: unreachable();
    }
}
