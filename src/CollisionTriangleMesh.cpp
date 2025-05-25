#include "CollisionTriangleMesh.h"
#include "BoundsUtil.h"
#include "Clipping.h"
#include "GeomUtil.h"
#include <glm/ext/scalar_common.hpp>
#include <glm/gtx/norm.hpp>
#include "EPA.h"
#include "GJK.h"
#include "ContactsUtil.h"
#include <glm/gtx/string_cast.hpp>

using namespace physecs;

namespace {
    enum class BoxFeature: char { FACE, EDGE, UNKNOWN };

    struct BoxContactInfo {
        BoxFeature feature;
        char axis;
    };

    struct TriangleContactInfo {
        int triangleIndex;
        glm::vec3 normal;
        glm::vec3 closestPointBody;
        glm::vec3 closestPointTriangle;
        TriangleFeature feature;
        char featureIndex;
        float distance;
        BoxContactInfo boxContactInfo;

        bool operator<(const TriangleContactInfo &other) const {
            return distance < other.distance;
        }
    };
}

//Sphere

static bool collisionSphereTriangle(glm::vec3 pos0, float radius0, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 n, TriangleContactInfo& triangleContactInfo) {
    glm::vec3 u;
    TriangleFeature triangleFeature;
    char featureIndex;
    float sqrDist = sqrDistPointTriangle(pos0, a, b, c, u, triangleFeature, featureIndex);
    if (sqrDist > radius0 * radius0) return false;

    if (triangleFeature == TriangleFeature::FACE) {
        triangleContactInfo.normal = -n;
    }
    else {
        glm::vec3 normal = u - pos0;
        float nLen = glm::length(normal);

        if (nLen) {
            normal /= nLen;
            triangleContactInfo.normal = normal;
        }
        else triangleContactInfo.normal = -n;
    }

    triangleContactInfo.closestPointBody = pos0 + triangleContactInfo.normal * radius0;
    triangleContactInfo.closestPointTriangle = u;
    triangleContactInfo.feature = triangleFeature;
    triangleContactInfo.featureIndex = featureIndex;
    triangleContactInfo.distance = sqrDist;
    return true;
}

static void generateContactsSphereTriangle(glm::vec3 pos1, glm::quat or1, const TriangleContactInfo& contact, std::vector<ContactManifold> &results) {
    ContactManifold manifold;
    manifold.normal = or1 * contact.normal;
    manifold.numPoints = 1;
    manifold.points[0] = { pos1 + or1 * contact.closestPointBody, pos1 + or1 * contact.closestPointTriangle };
    manifold.triangleIndex = contact.triangleIndex;

    results.push_back(manifold);
}

//capsule

static bool collisionCapsuleTriangle(glm::vec3 pos, glm::quat ori, float halfHeight, float radius, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 n, TriangleContactInfo& triangleContactInfo) {
    glm::vec3 dir = ori * glm::vec3(0, 1, 0);

    float t;
    glm::vec3 u;
    TriangleFeature triangleFeature;
    char featureIndex;
    float sqrDist = sqrDistSegmentTriangle(pos, dir, -halfHeight, halfHeight, a, b, c, t, u, triangleFeature, featureIndex);

    if (sqrDist > radius * radius) return false;

    glm::vec3 onSegment = pos + dir * t;

    if (triangleFeature == TriangleFeature::FACE) {
        triangleContactInfo.normal = -n;
    }
    else {
        glm::vec3 normal = u - onSegment;
        float nLen = glm::length(normal);

        if (nLen) {
            normal /= nLen;
            triangleContactInfo.normal = normal;
        }
        else triangleContactInfo.normal = -n;
    }

    triangleContactInfo.closestPointBody = onSegment + triangleContactInfo.normal * radius;
    triangleContactInfo.closestPointTriangle = u;
    triangleContactInfo.feature = triangleFeature;
    triangleContactInfo.featureIndex = featureIndex;
    triangleContactInfo.distance = sqrDist;

    return true;
}

static void generateContactsCapsuleTriangleFace(glm::vec3 pos0, glm::quat or0, float halfHeight0, float radius0, glm::vec3 pos1, glm::quat or1, TriangleMesh* mesh1, const TriangleContactInfo& contact, std::vector<ContactManifold> &results) {
    glm::vec3 dir = or0 * glm::vec3(0, 1, 0);

    glm::vec3 p0local = pos0 - dir * halfHeight0;
    glm::vec3 p1local = pos0 + dir * halfHeight0;

    auto& tri = mesh1->triangles[contact.triangleIndex];

    glm::vec3 refPlaneOrigin = tri.centroid;

    glm::vec3 u0 = glm::normalize(mesh1->vertices[tri.indices[0]] - refPlaneOrigin);
    glm::vec3 u1 = glm::normalize(tri.normal);
    glm::vec3 u2 = glm::cross(u0, u1);

    glm::mat3 meshToRef = glm::transpose(glm::mat3(u0, u1, u2));

    const int faceAxis = 1;
    const int clipX = 2;
    const int clipY = 0;

    glm::vec2 clip[3];
    for (int i = 0; i < 3; i++) {
        auto vertex = meshToRef * mesh1->vertices[tri.indices[i]];
        clip[3 - i - 1] = { vertex[clipX], vertex[clipY] };
    }

    glm::vec3 p0Ref = meshToRef * p0local;
    glm::vec3 p1Ref = meshToRef * p1local;

    glm::vec2 line0 = { p0Ref[clipX], p0Ref[clipY] };
    glm::vec2 line1 = { p1Ref[clipX], p1Ref[clipY] };

    if (!clipLine(line0, line1, clip, 3)) return;

    glm::mat3 meshToWorld = glm::mat3(or1);

    glm::mat3 refToWorld = meshToWorld * glm::transpose(meshToRef);
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

    p0[faceAxis] -= radius0;
    p1[faceAxis] -= radius0;

    int numPoints = 0;

    ContactManifold result;
    result.triangleIndex = contact.triangleIndex;
    result.normal = meshToWorld * contact.normal;

    if (p0[faceAxis] < distToPlane) {
        p0[clipX] = line0.x;
        p0[clipY] = line0.y;

        glm::vec3 onFace = p0;
        onFace[faceAxis] = distToPlane;

        result.points[numPoints++] = { pos1 + refToWorld * p0, pos1 + refToWorld * onFace };
    }

    if (p1[faceAxis] < distToPlane) {
        p1[clipX] = line1.x;
        p1[clipY] = line1.y;

        glm::vec3 onFace = p1;
        onFace[faceAxis] = distToPlane;

        result.points[numPoints++] = { pos1 + refToWorld * p1, pos1 + refToWorld * onFace };
    }

    if (!numPoints) {
        numPoints = 1;
        result.points[0] = { pos1 + meshToWorld * contact.closestPointBody, pos1 + meshToWorld * contact.closestPointTriangle };
    }

    result.numPoints = numPoints;

    results.push_back(result);
}

//Box

static bool collisionBoxTriangle(glm::vec3 pos, glm::quat ori, glm::vec3 halfExtents, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 n, TriangleContactInfo& triangleContactInfo) {
    float edgeOffset = 0.f;
    const float edgeLimit = 0.999f;

    glm::mat3 u = glm::mat3(ori);
    glm::mat3 invU = glm::transpose(u);

    glm::vec3 v0 = invU * (a - pos);
    glm::vec3 v1 = invU * (b - pos);
    glm::vec3 v2 = invU * (c - pos);

    glm::vec3 f0 = v1 - v0;
    glm::vec3 f1 = v2 - v1;
    glm::vec3 f2 = v0 - v2;

    float f0Limit = edgeLimit * glm::length2(f0);
    float f1Limit = edgeLimit * glm::length2(f1);
    float f2Limit = edgeLimit * glm::length2(f2);

    float p0, p1, r, d;

    float max = std::numeric_limits<float>::lowest();

    //edge vs edge

    //axis u0xf0
    p0 = v0.z * f0.y - v0.y * f0.z;
    p1 = v2.z * f0.y - v2.y * f0.z;
    r = halfExtents.y * glm::abs(f0.z) + halfExtents.z * glm::abs(f0.y);
    d = glm::max(-glm::max(p0, p1), glm::min(p0, p1)) - r;
    if (d > 0) return false;
    if (f0.x * f0.x < f0Limit && d > max) {
        max = d;
        triangleContactInfo.feature = TriangleFeature::EDGE;
        triangleContactInfo.featureIndex = 2;
        triangleContactInfo.distance = d;
        triangleContactInfo.boxContactInfo.feature = BoxFeature::EDGE;
        triangleContactInfo.boxContactInfo.axis = 0;
        edgeOffset = 0.1f;
    }

    //axis u0xf1
    p0 = v0.z * f1.y - v0.y * f1.z;
    p1 = v1.z * f1.y - v1.y * f1.z;
    r = halfExtents.y * glm::abs(f1.z) + halfExtents.z * glm::abs(f1.y);
    d = glm::max(-glm::max(p0, p1), glm::min(p0, p1)) - r;
    if (d > 0) return false;
    if (f1.x * f1.x < f1Limit && d > max) {
        max = d;
        triangleContactInfo.feature = TriangleFeature::EDGE;
        triangleContactInfo.featureIndex = 0;
        triangleContactInfo.distance = d;
        triangleContactInfo.boxContactInfo.feature = BoxFeature::EDGE;
        triangleContactInfo.boxContactInfo.axis = 0;
        edgeOffset = 0.1f;
    }

    //axis u0xf2
    p0 = v1.z * f2.y - v1.y * f2.z;
    p1 = v2.z * f2.y - v2.y * f2.z;
    r = halfExtents.y * glm::abs(f2.z) + halfExtents.z * glm::abs(f2.y);
    d = glm::max(-glm::max(p0, p1), glm::min(p0, p1)) - r;
    if (d > 0) return false;
    if (f2.x * f2.x < f2Limit && d > max) {
        max = d;
        triangleContactInfo.feature = TriangleFeature::EDGE;
        triangleContactInfo.featureIndex = 1;
        triangleContactInfo.distance = d;
        triangleContactInfo.boxContactInfo.feature = BoxFeature::EDGE;
        triangleContactInfo.boxContactInfo.axis = 0;
        edgeOffset = 0.1f;
    }

    //axis u1xf0
    p0 = v0.x * f0.z - v0.z * f0.x;
    p1 = v2.x * f0.z - v2.z * f0.x;
    r = halfExtents.x * glm::abs(f0.z) + halfExtents.z * glm::abs(f0.x);
    d = glm::max(-glm::max(p0, p1), glm::min(p0, p1)) - r;
    if (d > 0) return false;
    if (f0.y * f0.y < f0Limit && d > max) {
        max = d;
        triangleContactInfo.feature = TriangleFeature::EDGE;
        triangleContactInfo.featureIndex = 2;
        triangleContactInfo.distance = d;
        triangleContactInfo.boxContactInfo.feature = BoxFeature::EDGE;
        triangleContactInfo.boxContactInfo.axis = 1;
        edgeOffset = 0.1f;
    }

    //axis u1xf1
    p0 = v0.x * f1.z - v0.z * f1.x;
    p1 = v1.x * f1.z - v1.z * f1.x;
    r = halfExtents.x * glm::abs(f1.z) + halfExtents.z * glm::abs(f1.x);
    d = glm::max(-glm::max(p0, p1), glm::min(p0, p1)) - r;
    if (d > 0) return false;
    if (f1.y * f1.y < f1Limit && d > max) {
        max = d;
        triangleContactInfo.feature = TriangleFeature::EDGE;
        triangleContactInfo.featureIndex = 0;
        triangleContactInfo.distance = d;
        triangleContactInfo.boxContactInfo.feature = BoxFeature::EDGE;
        triangleContactInfo.boxContactInfo.axis = 1;
        edgeOffset = 0.1f;
    }

    //axis u1xf2
    p0 = v1.x * f2.z - v1.z * f2.x;
    p1 = v2.x * f2.z - v2.z * f2.x;
    r = halfExtents.x * glm::abs(f2.z) + halfExtents.z * glm::abs(f2.x);
    d = glm::max(-glm::max(p0, p1), glm::min(p0, p1)) - r;
    if (d > 0) return false;
    if (f2.y * f2.y < f2Limit && d > max) {
        max = d;
        triangleContactInfo.feature = TriangleFeature::EDGE;
        triangleContactInfo.featureIndex = 1;
        triangleContactInfo.distance = d;
        triangleContactInfo.boxContactInfo.feature = BoxFeature::EDGE;
        triangleContactInfo.boxContactInfo.axis = 1;
        edgeOffset = 0.1f;
    }

    //axis u2xf0
    p0 = v0.y * f0.x - v0.x * f0.y;
    p1 = v2.y * f0.x - v2.x * f0.y;
    r = halfExtents.x * glm::abs(f0.y) + halfExtents.y * glm::abs(f0.x);
    d = glm::max(-glm::max(p0, p1), glm::min(p0, p1)) - r;
    if (d > 0) return false;
    if (f0.z * f0.z < f0Limit && d > max) {
        max = d;
        triangleContactInfo.feature = TriangleFeature::EDGE;
        triangleContactInfo.featureIndex = 2;
        triangleContactInfo.distance = d;
        triangleContactInfo.boxContactInfo.feature = BoxFeature::EDGE;
        triangleContactInfo.boxContactInfo.axis = 2;
        edgeOffset = 0.1f;
    }

    //axis u2xf1
    p0 = v0.y * f1.x - v0.x * f1.y;
    p1 = v1.y * f1.x - v1.x * f1.y;
    r = halfExtents.x * glm::abs(f1.y) + halfExtents.y * glm::abs(f1.x);
    d = glm::max(-glm::max(p0, p1), glm::min(p0, p1)) - r;
    if (d > 0) return false;
    if (f1.z * f1.z < f1Limit && d > max) {
        max = d;
        triangleContactInfo.feature = TriangleFeature::EDGE;
        triangleContactInfo.featureIndex = 0;
        triangleContactInfo.distance = d;
        triangleContactInfo.boxContactInfo.feature = BoxFeature::EDGE;
        triangleContactInfo.boxContactInfo.axis = 2;
        edgeOffset = 0.1f;
    }

    //axis u2xf2
    p0 = v1.y * f2.x - v1.x * f2.y;
    p1 = v2.y * f2.x - v2.x * f2.y;
    r = halfExtents.x * glm::abs(f2.y) + halfExtents.y * glm::abs(f2.x);
    d = glm::max(-glm::max(p0, p1), glm::min(p0, p1)) - r;
    if (d > 0) return false;
    if (f2.z * f2.z < f2Limit && d > max) {
        max = d;
        triangleContactInfo.feature = TriangleFeature::EDGE;
        triangleContactInfo.featureIndex = 1;
        triangleContactInfo.distance = d;
        triangleContactInfo.boxContactInfo.feature = BoxFeature::EDGE;
        triangleContactInfo.boxContactInfo.axis = 2;
        edgeOffset = 0.1f;
    }

    //box face normals

    for (int i = 0; i < 3; ++i) {
        d = glm::max(-glm::max(v0[i], v1[i], v2[i]), glm::min(v0[i], v1[i], v2[i])) - halfExtents[i];
        if (d > 0) return false;
        if (d > max - edgeOffset) {
            max = d;

            if (glm::abs(v0[i]) < halfExtents[i]) {
                if (glm::abs(v1[i]) < halfExtents[i]) {
                    if (glm::abs(v2[i]) < halfExtents[i]) {
                        triangleContactInfo.feature = TriangleFeature::FACE;
                    }
                    else {
                        triangleContactInfo.feature = TriangleFeature::EDGE;
                        triangleContactInfo.featureIndex = 2;
                    }
                }
                else {
                    if (glm::abs(v2[i]) < halfExtents[i]) {
                        triangleContactInfo.feature = TriangleFeature::EDGE;
                        triangleContactInfo.featureIndex = 1;
                    }
                    else {
                        triangleContactInfo.feature = TriangleFeature::VERTEX;
                        triangleContactInfo.featureIndex = 0;
                    }
                }
            }
            else {
                if (glm::abs(v1[i]) < halfExtents[i]) {
                    if (glm::abs(v2[i]) < halfExtents[i]) {
                        triangleContactInfo.feature = TriangleFeature::EDGE;
                        triangleContactInfo.featureIndex = 0;
                    }
                    else {
                        triangleContactInfo.feature = TriangleFeature::VERTEX;
                        triangleContactInfo.featureIndex = 1;
                    }
                }
                else {
                    triangleContactInfo.feature = TriangleFeature::VERTEX;
                    triangleContactInfo.featureIndex = 2;
                }
            }

            triangleContactInfo.boxContactInfo.feature = BoxFeature::FACE;
            triangleContactInfo.boxContactInfo.axis = i;

            triangleContactInfo.distance = d;
            edgeOffset = 0.f;
        }
    }

    //triangle normal

    glm::vec3 nLocal = invU * n;
    d = distanceAABBPlane(halfExtents, nLocal, glm::dot(nLocal, v0));
    if (d > 0) return false;
    if (d > max - edgeOffset) {
        triangleContactInfo.feature = TriangleFeature::FACE;
        triangleContactInfo.distance = d;
        triangleContactInfo.boxContactInfo.feature = BoxFeature::UNKNOWN;
    }

    return true;
}

static void generateContactsBoxTriangleFace(glm::vec3 boxPos, glm::quat boxOr, glm::vec3 boxHalfExtents, glm::vec3 meshPos, glm::quat meshOr, TriangleMesh* mesh, const TriangleContactInfo& contact, std::vector<ContactManifold>& results) {
    glm::mat3 boxBasis = glm::mat3(boxOr);
    auto& tri = mesh->triangles[contact.triangleIndex];

    glm::vec3 refPlaneOrigin = tri.centroid;

    glm::vec3 u0 = glm::normalize(mesh->vertices[tri.indices[0]] - refPlaneOrigin);
    glm::vec3 u1 = glm::normalize(tri.normal);
    glm::vec3 u2 = glm::cross(u0, u1);

    glm::mat3 meshToRef = glm::transpose(glm::mat3(u0, u1, u2));

    const int clipX = 2;
    const int clipY = 0;

    std::vector<glm::vec2> clip(3);
    for (int i = 0; i < 3; i++) {
        auto vertex = meshToRef * mesh->vertices[tri.indices[i]];
        clip[3 - i - 1] = { vertex[clipX], vertex[clipY] };
    }

    int incAxis;
    float incSign;
    if (contact.boxContactInfo.feature == BoxFeature::FACE) {
        incAxis = contact.boxContactInfo.axis;
        incSign = glm::dot(tri.normal, boxBasis[incAxis]) > 0 ? -1 : 1;
    }
    else {
        float maxDot = 0;
        for (int i = 0; i < 3; ++i) {
            float dot = glm::dot(tri.normal, boxBasis[i]);
            float absDot = glm::abs(dot);
            if (absDot > maxDot) {
                maxDot = absDot;
                incAxis = i;
                incSign = dot > 0 ? -1 : 1;
            }
        }
    }

    int incAxis1 = (incAxis + 1) % 3, incAxis2 = (incAxis + 2) % 3;

    glm::vec3 incPlaneOrig = meshToRef * (boxPos + boxBasis[incAxis] * boxHalfExtents[incAxis] * incSign);

    auto poly0 = incPlaneOrig + meshToRef * (boxBasis[incAxis1] * boxHalfExtents[incAxis1] + boxBasis[incAxis2] * boxHalfExtents[incAxis2]);
    auto poly1 = incPlaneOrig + meshToRef * (-boxBasis[incAxis1] * boxHalfExtents[incAxis1] + boxBasis[incAxis2] * boxHalfExtents[incAxis2]);
    auto poly2 = incPlaneOrig + meshToRef * (-boxBasis[incAxis1] * boxHalfExtents[incAxis1] - boxBasis[incAxis2] * boxHalfExtents[incAxis2]);
    auto poly3 = incPlaneOrig + meshToRef * (boxBasis[incAxis1] * boxHalfExtents[incAxis1] - boxBasis[incAxis2] * boxHalfExtents[incAxis2]);

    std::vector<glm::vec2> polygon;
    polygon.reserve(6);
    polygon.resize(4);

    polygon[0] = {poly0[clipX], poly0[clipY]};
    polygon[1] = {poly1[clipX], poly1[clipY]};
    polygon[2] = {poly2[clipX], poly2[clipY]};
    polygon[3] = {poly3[clipX], poly3[clipY]};

    suthHodgClip(polygon, clip);

    glm::vec3 incPlaneNormal = meshToRef * boxBasis[incAxis];

    glm::mat3 meshToWorld = glm::mat3(meshOr);
    glm::mat3 refToWorld = meshToWorld * glm::transpose(meshToRef);

    ContactManifold manifold;
    manifold.normal = -meshToWorld * tri.normal;
    manifold.triangleIndex = contact.triangleIndex;
    generateContactsPolygonPolygonFace(meshPos, refToWorld, refPlaneOrigin, u1, incPlaneOrig, incPlaneNormal, polygon, clipX, clipY, manifold.points, manifold.numPoints);
    for (int i = 0; i < manifold.numPoints; ++i) {
        manifold.points[i] = { manifold.points[i].position1, manifold.points[i].position0 };
    }
    results.push_back(manifold);
}

void generateContactsBoxFaceTriangle(glm::vec3 boxPos, glm::quat boxOr, glm::vec3 boxHalfExtents, glm::vec3 meshPos, glm::quat meshOr, TriangleMesh* mesh, const TriangleContactInfo& contact, std::vector<ContactManifold>& results) {
    glm::mat3 boxBasis = glm::mat3(boxOr);
    auto& tri = mesh->triangles[contact.triangleIndex];

    glm::mat3 meshToBox = glm::transpose(boxBasis);

    int refAxis = contact.boxContactInfo.axis;
    float refSign = glm::dot(boxBasis[refAxis], tri.centroid - boxPos) < 0 ? -1 : 1;
    int clipX = (refAxis + 1) % 3, clipY = (refAxis + 2) % 3;

    std::vector<glm::vec2> polygon(3);
    for (int i = 0; i < 3; ++i) {
        int index = tri.indices[i];
        auto vertex = meshToBox * (mesh->vertices[index] - boxPos);
        polygon[i] = { vertex[clipX], vertex[clipY] };
    }

    std::vector<glm::vec2> clip(4);
    clip[0] = {boxHalfExtents[clipX], boxHalfExtents[clipY]};
    clip[1] = {boxHalfExtents[clipX], -boxHalfExtents[clipY]};
    clip[2] = {-boxHalfExtents[clipX], -boxHalfExtents[clipY]};
    clip[3] = {-boxHalfExtents[clipX], boxHalfExtents[clipY]};

    suthHodgClip(polygon, clip);

    glm::vec3 incPlaneOrigin = meshToBox * (tri.centroid - boxPos);
    glm::vec3 incPlaneNormal = meshToBox * tri.normal;

    glm::mat3 meshToWorld = glm::mat3(meshOr);

    ContactManifold manifold;
    manifold.normal = meshToWorld * (refSign * boxBasis[refAxis]);
    manifold.triangleIndex = contact.triangleIndex;
    generateContactsPolygonBoxFace(boxPos, boxBasis, refAxis, refSign, boxHalfExtents, incPlaneOrigin, incPlaneNormal, polygon, clipX, clipY, manifold.points, manifold.numPoints);
    for (int i = 0; i < manifold.numPoints; ++i) {
        manifold.points[i].position0 = meshPos + meshToWorld * manifold.points[i].position0;
        manifold.points[i].position1 = meshPos + meshToWorld * manifold.points[i].position1;
    }
    results.push_back(manifold);
}

void generateContactsBoxEdgeTriangleEdge(glm::vec3 boxPos, glm::quat boxOr, glm::vec3 boxHalfExtents, glm::vec3 meshPos, glm::quat meshOr, TriangleMesh* mesh, const TriangleContactInfo& contact, std::vector<ContactManifold>& results) {
    glm::mat3 boxBasis = glm::mat3(boxOr);
    auto& tri = mesh->triangles[contact.triangleIndex];

    int boxAxis = contact.boxContactInfo.axis;
    int triangleEdge = contact.featureIndex;

    glm::vec3 triA = mesh->vertices[tri.indices[(triangleEdge + 1) % 3]];
    glm::vec3 triB = mesh->vertices[tri.indices[(triangleEdge + 2) % 3]];
    glm::vec3 triEdge = triB - triA;

    glm::vec3 axis = glm::cross(boxBasis[boxAxis], triEdge);

    glm::mat3 meshToWorld = glm::mat3(meshOr);

    ContactManifold manifold;
    manifold.normal = meshToWorld * glm::normalize(glm::dot(axis, tri.centroid - boxPos) < 0 ? -axis : axis);
    manifold.numPoints = 1;
    manifold.triangleIndex = contact.triangleIndex;

    glm::vec3 signBox(-1);
    signBox[(boxAxis + 1) % 3] = glm::dot(boxBasis[(boxAxis + 1) % 3], manifold.normal) < 0 ? -1 : 1;
    signBox[(boxAxis + 2) % 3] = glm::dot(boxBasis[(boxAxis + 2) % 3], manifold.normal) < 0 ? -1 : 1;
    glm::vec3 boxA = boxPos + signBox[0] * boxHalfExtents[0] * boxBasis[0] + signBox[1] * boxHalfExtents[1] * boxBasis[1] + signBox[2] * boxHalfExtents[2] * boxBasis[2];
    glm::vec3 boxEdge = boxBasis[boxAxis] * boxHalfExtents[boxAxis] * 2.f;
    auto [p0, p1] = closestPointsBetweenSegments(boxA, boxEdge, triA, triEdge);
    manifold.points[0] = { meshPos + meshToWorld * p0, meshPos + meshToWorld * p1 };
    results.push_back(manifold);
}

void generateContactsBoxTriangleEdge(glm::vec3 boxPos, glm::quat boxOr, glm::vec3 boxHalfExtents, glm::vec3 meshPos, glm::quat meshOr, TriangleMesh* mesh, const TriangleContactInfo& contact, std::vector<ContactManifold>& results) {
    if (contact.boxContactInfo.feature == BoxFeature::FACE) {
        generateContactsBoxFaceTriangle(boxPos, boxOr, boxHalfExtents, meshPos, meshOr, mesh, contact, results);
    }
    else {
        generateContactsBoxEdgeTriangleEdge(boxPos, boxOr, boxHalfExtents, meshPos, meshOr, mesh, contact, results);
    }
}

//Convex mesh

static bool collisionConvexMeshTriangle(glm::vec3 pos, glm::quat ori, ConvexMesh* mesh, glm::vec3 scale, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 n, glm::vec3 centroid, TriangleContactInfo& triangleContactInfo) {
    ConvexMeshSupportFunction convexMeshSupport{ pos, glm::toMat3(ori), mesh->vertices, scale };
    TriangleSupportFunction triangleSupport{ a, b, c };

    GjkSimplex s;
    if (!GJK<ConvexMeshSupportFunction, TriangleSupportFunction>::gjk(convexMeshSupport, triangleSupport, centroid - pos, s)) return false;

    triangleContactInfo.normal = EPA<ConvexMeshSupportFunction, TriangleSupportFunction>::epa(convexMeshSupport, triangleSupport, s, triangleContactInfo.closestPointBody, triangleContactInfo.closestPointTriangle);
    triangleContactInfo.distance = glm::dot(triangleContactInfo.closestPointTriangle - triangleContactInfo.closestPointBody, triangleContactInfo.normal);

    //compute barycentric coords for closest point on triangle

    glm::vec3 v0 = b - a;
    glm::vec3 v1 = c - a;
    glm::vec3 v2 = triangleContactInfo.closestPointTriangle - a;

    float d00 = glm::dot(v0, v0);
    float d01 = glm::dot(v0, v1);
    float d11 = glm::dot(v1, v1);
    float d20 = glm::dot(v2, v0);
    float d21 = glm::dot(v2, v1);

    float denom = d00 * d11 - d01 * d01;

    float u, v, w;

    if (denom) {
        v = (d20 * d11 - d21 * d01) / denom;
        w = (d00 * d21 - d01 * d20) / denom;
        u = 1.0f - v - w;
    }
    else {
        w = 0;
        if (d00) {
            v = d20 / d00;
            u = 1.0f - v;
        }
        else {
            v = 0;
            u = 1.0;
        }
    }

    float epsilon = 1e-4;
    if (u < epsilon) {
        if (v < epsilon) {
            triangleContactInfo.feature = TriangleFeature::VERTEX;
            triangleContactInfo.featureIndex = 2;
        }
        else if (w < epsilon) {
            triangleContactInfo.feature = TriangleFeature::VERTEX;
            triangleContactInfo.featureIndex = 1;
        }
        else {
            triangleContactInfo.feature = TriangleFeature::EDGE;
            triangleContactInfo.featureIndex = 0;
        }
    }
    else {
        if (v < epsilon) {
            if (w < epsilon) {
                triangleContactInfo.feature = TriangleFeature::VERTEX;
                triangleContactInfo.featureIndex = 0;
            }
            else {
                triangleContactInfo.feature = TriangleFeature::EDGE;
                triangleContactInfo.featureIndex = 1;
            }
        }
        else if (w < epsilon) {
            if (u < epsilon) {
                triangleContactInfo.feature = TriangleFeature::VERTEX;
                triangleContactInfo.featureIndex = 0;
            }
            else {
                triangleContactInfo.feature = TriangleFeature::EDGE;
                triangleContactInfo.featureIndex = 2;
            }
        }
        else {
            triangleContactInfo.feature = TriangleFeature::FACE;
        }
    }

    return true;
}

void generateContactsConvexMeshTriangleFace(glm::vec3 convexPos, glm::quat convexOr, ConvexMesh* convex, glm::vec3 scale, glm::vec3 meshPos, glm::quat meshOr, TriangleMesh* mesh, const TriangleContactInfo& contact, std::vector<ContactManifold>& results) {
    auto& tri = mesh->triangles[contact.triangleIndex];

    glm::vec3 refPlaneOrigin = tri.centroid;

    glm::vec3 u0 = glm::normalize(mesh->vertices[tri.indices[0]] - refPlaneOrigin);
    glm::vec3 u1 = tri.normal;
    glm::vec3 u2 = glm::cross(u0, u1);

    glm::mat3 meshToRef = glm::transpose(glm::mat3(u0, u1, u2));

    const int clipX = 2;
    const int clipY = 0;

    std::vector<glm::vec2> clip(3);
    for (int i = 0; i < 3; i++) {
        auto vertex = meshToRef * mesh->vertices[tri.indices[i]];
        clip[3 - i - 1] = { vertex[clipX], vertex[clipY] };
    }

    int convexFaceIndex;
    float minDot = 0;
    for (int i = 0; i < convex->faces.size(); ++i) {
        float dot = glm::dot(tri.normal, glm::normalize(convexOr * (convex->faces[i].normal / scale)));
        if (dot < minDot) {
            minDot = dot;
            convexFaceIndex = i;
        }
    }
    auto& convexFace = convex->faces[convexFaceIndex];

    std::vector<glm::vec2> polygon(convexFace.indices.size());
    for (int i = 0; i < convexFace.indices.size(); ++i) {
        int index = convexFace.indices[i];
        auto vertex = meshToRef * (convexPos + convexOr * (scale * convex->vertices[index]));
        polygon[i] = { vertex[clipX], vertex[clipY] };
    }

    suthHodgClip(polygon, clip);

    glm::vec3 incPlaneOrigin = meshToRef * (convexPos + convexOr * (scale * convexFace.centroid));
    glm::vec3 incPlaneNormal = meshToRef * (convexOr * glm::normalize(convexFace.normal / scale));

    glm::mat3 meshToWorld = glm::mat3(meshOr);
    glm::mat3 refToWorld = meshToWorld * glm::transpose(meshToRef);

    ContactManifold manifold;
    manifold.normal = -meshToWorld * tri.normal;
    manifold.triangleIndex = contact.triangleIndex;
    generateContactsPolygonPolygonFace(meshPos, refToWorld, refPlaneOrigin, u1, incPlaneOrigin, incPlaneNormal, polygon, clipX, clipY, manifold.points, manifold.numPoints);
    for (int i = 0; i < manifold.numPoints; ++i) {
        manifold.points[i] = { manifold.points[i].position1, manifold.points[i].position0 };
    }
    results.push_back(manifold);
}

static void generateContactsConvexFaceTriangle(glm::vec3 convexPos, glm::quat convexOr, ConvexMesh* convex, glm::vec3 scale, glm::vec3 meshPos, glm::quat meshOr, TriangleMesh* mesh, const TriangleContactInfo& contact, std::vector<ContactManifold>& results) {
    auto& tri = mesh->triangles[contact.triangleIndex];

    glm::mat3 convexToMesh = glm::mat3(convexOr);
    glm::mat3 meshToConvex = glm::transpose(convexToMesh);

    int convexFaceIndex;
    float maxDot = 0;
    for (int i = 0; i < convex->faces.size(); ++i) {
        float dot = glm::dot(contact.normal, glm::normalize(convexOr * (convex->faces[i].normal / scale)));
        if (dot > maxDot) {
            maxDot = dot;
            convexFaceIndex = i;
        }
    }
    auto& convexFace = convex->faces[convexFaceIndex];

    glm::vec3 refPlaneOrigin = scale * convexFace.centroid;

    glm::vec3 u0 = glm::normalize(scale * convex->vertices[convexFace.indices[0]] - refPlaneOrigin);
    glm::vec3 u1 = glm::normalize(convexFace.normal / scale);
    glm::vec3 u2 = glm::cross(u0, u1);

    glm::mat3 convexToRef = glm::transpose(glm::mat3(u0, u1, u2));
    glm::mat3 meshToRef = convexToRef * meshToConvex;;

    int clipX = 2;
    int clipY = 0;

    std::vector<glm::vec2> clip(convexFace.indices.size());
    for (int i = 0; i < convexFace.indices.size(); ++i) {
        int index = convexFace.indices[i];
        auto vertex = convexToRef * (scale * convex->vertices[index]);
        clip[i] = { vertex[clipX], vertex[clipY] };
    }

    std::vector<glm::vec2> polygon(3);
    for (int i = 0; i < 3; ++i) {
        int index = tri.indices[i];
        auto vertex = meshToRef * (mesh->vertices[index] - convexPos);
        polygon[i] = { vertex[clipX], vertex[clipY] };
    }

    suthHodgClip(polygon, clip);

    glm::vec3 incPlaneOrigin = meshToRef * (tri.centroid - convexPos);
    glm::vec3 incPlaneNormal = meshToRef * tri.normal;

    glm::mat3 meshToWorld = glm::mat3(meshOr);
    glm::mat3 refToMesh = glm::transpose(meshToRef);

    ContactManifold manifold;
    manifold.triangleIndex = contact.triangleIndex;
    generateContactsPolygonPolygonFace(convexPos, refToMesh, refPlaneOrigin, u1, incPlaneOrigin, incPlaneNormal, polygon, clipX, clipY, manifold.points, manifold.numPoints);
    if (manifold.numPoints) {
        for (int i = 0; i < manifold.numPoints; ++i) {
            manifold.points[i].position0 = meshPos + meshToWorld * manifold.points[i].position0;
            manifold.points[i].position1 = meshPos + meshToWorld * manifold.points[i].position1;
        }
        manifold.normal = meshToWorld * convexToMesh * u1;
    }
    else {
        manifold.normal = meshToWorld * contact.normal;
        manifold.numPoints = 1;
        manifold.points[0] = { meshPos + meshToWorld * contact.closestPointBody, meshPos + meshToWorld * contact.closestPointTriangle };
    }
    results.push_back(manifold);
}

//General

static bool collisionTriangle(glm::vec3 pos, glm::quat ori, const Geometry& geom, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 n, glm::vec3 centroid, TriangleContactInfo& triangleContactInfo) {
    switch (geom.type) {
        case SPHERE:
            return collisionSphereTriangle(pos, geom.sphere.radius, a, b, c, n, triangleContactInfo);
        case CAPSULE:
            return collisionCapsuleTriangle(pos, ori, geom.capsule.halfHeight, geom.capsule.radius, a, b, c, n, triangleContactInfo);
        case BOX:
            return collisionBoxTriangle(pos, ori, geom.box.halfExtents, a, b, c, n, triangleContactInfo);
        case CONVEX_MESH:
            return collisionConvexMeshTriangle(pos, ori, geom.convex.mesh, geom.convex.scale, a, b, c, n, centroid, triangleContactInfo);
    }

    return false;
}

static void generateContactsTriangleFace(glm::vec3 pos0, glm::quat or0, const Geometry& geom0, glm::vec3 pos1, glm::quat or1, TriangleMesh* mesh1, const TriangleContactInfo& contact, std::vector<ContactManifold> &results) {
    switch (geom0.type) {
        case SPHERE:
            return generateContactsSphereTriangle(pos1, or1, contact, results);
        case CAPSULE:
            return generateContactsCapsuleTriangleFace(pos0, or0, geom0.capsule.halfHeight, geom0.capsule.radius, pos1, or1, mesh1, contact, results);
        case BOX:
            return generateContactsBoxTriangleFace(pos0, or0, geom0.box.halfExtents, pos1, or1, mesh1, contact, results);
        case CONVEX_MESH:
            return generateContactsConvexMeshTriangleFace(pos0, or0, geom0.convex.mesh, geom0.convex.scale, pos1, or1, mesh1, contact, results);
    }
}

static void generateContactsTriangleEdge(glm::vec3 pos0, glm::quat or0, const Geometry& geom0, glm::vec3 pos1, glm::quat or1, TriangleMesh* mesh1, const TriangleContactInfo& contact, std::vector<ContactManifold> &results) {
    switch (geom0.type) {
        case SPHERE:
            return generateContactsSphereTriangle(pos1, or1, contact, results);
        case CAPSULE:
            return generateContactsSphereTriangle(pos1, or1, contact, results);
        case BOX:
            return generateContactsBoxTriangleEdge(pos0, or0, geom0.box.halfExtents, pos1, or1, mesh1, contact, results);
        case CONVEX_MESH:
            return generateContactsConvexFaceTriangle(pos0, or0, geom0.convex.mesh, geom0.convex.scale, pos1, or1, mesh1, contact, results);
    }
}

static void generateContactsTriangleVertex(glm::vec3 pos0, glm::quat or0, const Geometry& geom0, glm::vec3 pos1, glm::quat or1, TriangleMesh* mesh1, const TriangleContactInfo& contact, std::vector<ContactManifold> &results) {
    switch (geom0.type) {
        case SPHERE:
            return generateContactsSphereTriangle(pos1, or1, contact, results);
        case CAPSULE:
            return generateContactsSphereTriangle(pos1, or1, contact, results);
        case BOX:
            return generateContactsBoxFaceTriangle(pos0, or0, geom0.box.halfExtents, pos1, or1, mesh1, contact, results);;
        case CONVEX_MESH:
            return generateContactsConvexFaceTriangle(pos0, or0, geom0.convex.mesh, geom0.convex.scale, pos1, or1, mesh1, contact, results);
    }
}

bool physecs::collisionTriangleMesh(glm::vec3 pos0, glm::quat or0, const Geometry& geom0, glm::vec3 pos1, glm::quat or1, TriangleMesh *mesh1, std::vector<ContactManifold> &results) {
    glm::quat invOr1 = glm::inverse(or1);
    glm::vec3 localPos = invOr1 * (pos0 - pos1);
    glm::quat localOr = invOr1 * or0;
    Bounds localBounds = getBounds(localPos, localOr, geom0);

    std::vector<int> potentialOverlaps = mesh1->overlapBvh(localBounds);

    if (potentialOverlaps.empty()) return false;

    std::vector<TriangleContactInfo> contacts;
    contacts.reserve(potentialOverlaps.size());

    for (int triangleIndex : potentialOverlaps) {
        auto tri = mesh1->triangles[triangleIndex];

        auto a = mesh1->vertices[tri.indices[0]];
        auto b = mesh1->vertices[tri.indices[1]];
        auto c = mesh1->vertices[tri.indices[2]];

        TriangleContactInfo triangleContactInfo;
        if (collisionTriangle(localPos, localOr, geom0, a, b, c, tri.normal, tri.centroid, triangleContactInfo)) {
            triangleContactInfo.triangleIndex = triangleIndex;
            contacts.push_back(triangleContactInfo);
        }
    }

    if (contacts.empty()) return false;

    std::unordered_set<unsigned int> voidedVertices;

    //first pass: face contacts
    for (int i = contacts.size() - 1; i >= 0; i--) {
        auto& contact = contacts[i];
        if (contact.feature == TriangleFeature::FACE) {
            auto& tri = mesh1->triangles[contact.triangleIndex];

            voidedVertices.insert(tri.indices[0]);
            voidedVertices.insert(tri.indices[1]);
            voidedVertices.insert(tri.indices[2]);

            generateContactsTriangleFace(localPos, localOr, geom0, pos1, or1, mesh1, contact, results);

            //remove
            std::swap(contact, contacts.back());
            contacts.pop_back();
        }
    }

    //sort delayed edge and vertex contacts
    std::sort(contacts.begin(), contacts.end());

    //second pass: edge and vertex contacts
    for (auto& contact : contacts) {
        auto& tri = mesh1->triangles[contact.triangleIndex];

        switch (contact.feature) {
            case TriangleFeature::EDGE:
                if (voidedVertices.find(tri.indices[(contact.featureIndex + 1) % 3]) != voidedVertices.end() && voidedVertices.find(tri.indices[(contact.featureIndex + 2) % 3]) != voidedVertices.end())
                    continue;
                generateContactsTriangleEdge(localPos, localOr, geom0, pos1, or1, mesh1, contact, results);
            break;
            case TriangleFeature::VERTEX:
                if (voidedVertices.find(tri.indices[contact.featureIndex]) != voidedVertices.end())
                    continue;
                generateContactsTriangleVertex(localPos, localOr, geom0, pos1, or1, mesh1, contact, results);
        }

        voidedVertices.insert(tri.indices[0]);
        voidedVertices.insert(tri.indices[1]);
        voidedVertices.insert(tri.indices[2]);
    }

    return true;
}
