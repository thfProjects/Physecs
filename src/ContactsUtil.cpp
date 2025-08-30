#include "ContactsUtil.h"
#include <glm/gtx/norm.hpp>

namespace {
    thread_local std::vector<glm::vec3> penetratedPoints;
}

void physecs::generateContactsPolygonBoxFace(
    glm::vec3 boxCenter,
    glm::mat3 boxBasis,
    int boxAxis,
    int boxAxisSign,
    glm::vec3 boxHalfExtents,
    glm::vec3 incPlaneOrig,
    glm::vec3 incPlaneNormal,
    std::vector<glm::vec2>& polygon,
    int clipX,
    int clipY,
    ContactPoint* contactPoints,
    int& numPoints)
{
    penetratedPoints.clear();
    penetratedPoints.reserve(polygon.size());

    for (auto point: polygon) {
        glm::vec3 point3(0);
        point3[clipX] = point.x;
        point3[clipY] = point.y;
        glm::vec3 dir(0);
        dir[boxAxis] = boxAxisSign;
        float distance = glm::dot((incPlaneOrig - point3), incPlaneNormal) / glm::dot(dir, incPlaneNormal); //intersection of ray and plane
        if (distance < boxHalfExtents[boxAxis]) {
            point3[boxAxis] = boxAxisSign * distance;
            penetratedPoints.push_back(point3);
        }
    }

    if (penetratedPoints.size() > 4) {
        //build manifold from 4 points

        glm::vec3 penetratedPoint0 = penetratedPoints[0];
        penetratedPoint0[boxAxis] = boxAxisSign * boxHalfExtents[boxAxis];
        contactPoints[0] = {boxCenter + boxBasis * penetratedPoint0, boxCenter + boxBasis * penetratedPoints[0] };

        float maxDistance = 0;
        int maxDistanceIndex = 0;
        for (int i = 0; i < penetratedPoints.size(); ++i) {
            float distance = glm::distance2(penetratedPoints[0], penetratedPoints[i]);
            if (distance > maxDistance) {
                maxDistance = distance;
                maxDistanceIndex = i;
            }
        }

        penetratedPoint0 = penetratedPoints[maxDistanceIndex];
        penetratedPoint0[boxAxis] = boxAxisSign * boxHalfExtents[boxAxis];
        contactPoints[1] = {
            boxCenter + boxBasis * penetratedPoint0, boxCenter + boxBasis * penetratedPoints[maxDistanceIndex]
        };

        float maxArea = 0;
        int maxAreaIndex = 0;
        for (int i = 0; i < penetratedPoints.size(); ++i) {
            auto ca = penetratedPoints[0] - penetratedPoints[i];
            auto cb = penetratedPoints[maxDistanceIndex] - penetratedPoints[i];
            float area = glm::determinant(glm::mat2(glm::vec2(ca[clipX], ca[clipY]), glm::vec2(cb[clipX], cb[clipY])));
            if (area > maxArea) {
                maxArea = area;
                maxAreaIndex = i;
            }
        }

        penetratedPoint0 = penetratedPoints[maxAreaIndex];
        penetratedPoint0[boxAxis] = boxAxisSign * boxHalfExtents[boxAxis];
        contactPoints[2] = {
            boxCenter + boxBasis * penetratedPoint0, boxCenter + boxBasis * penetratedPoints[maxAreaIndex]
        };

        float minArea = 0;
        int minAreaIndex = 0;
        for (int i = 0; i < penetratedPoints.size(); ++i) {
            auto da = penetratedPoints[0] - penetratedPoints[i];
            auto db = penetratedPoints[maxDistanceIndex] - penetratedPoints[i];
            float area = glm::determinant(glm::mat2(glm::vec2(da[clipX], da[clipY]), glm::vec2(db[clipX], db[clipY])));
            if (area < minArea) {
                minArea = area;
                minAreaIndex = i;
            }
        }

        penetratedPoint0 = penetratedPoints[minAreaIndex];
        penetratedPoint0[boxAxis] = boxAxisSign * boxHalfExtents[boxAxis];
        contactPoints[3] = {
            boxCenter + boxBasis * penetratedPoint0, boxCenter + boxBasis * penetratedPoints[minAreaIndex]
        };

        numPoints = 4;
    } else {
        for (int i = 0; i < penetratedPoints.size(); ++i) {
            glm::vec3 penetratedPoint0 = penetratedPoints[i];
            penetratedPoint0[boxAxis] = boxAxisSign * boxHalfExtents[boxAxis];
            contactPoints[i] = { boxCenter + boxBasis * penetratedPoint0, boxCenter + boxBasis * penetratedPoints[i] };
        }
        numPoints = penetratedPoints.size();
    }
}

void physecs::generateContactsPolygonPolygonFace(
    glm::vec3 refPos,
    glm::mat3& refToWorld,
    glm::vec3 refPlaneOrigin,
    glm::vec3 refPlaneNormal,
    glm::vec3 incPlaneOrigin,
    glm::vec3 incPlaneNormal,
    std::vector<glm::vec2>& polygon,
    int clipX,
    int clipY,
    ContactPoint* contactPoints,
    int& numPoints)
{
    penetratedPoints.clear();
    penetratedPoints.reserve(polygon.size());

    float distanceToRefPlane = glm::dot(refPlaneOrigin, refPlaneNormal);
    for (auto point: polygon) {
        glm::vec3 point3(point.y, 0, point.x);
        glm::vec3 dir(0, 1, 0);
        float distance = glm::dot((incPlaneOrigin - point3), incPlaneNormal) / glm::dot(dir, incPlaneNormal); //intersection of ray and plane
        if (distance < distanceToRefPlane) {
            point3.y = distance;
            penetratedPoints.push_back(point3);
        }
    }

    if (penetratedPoints.size() > 4) {
        //build manifold from 4 points

        glm::vec3 penetratedPoint0 = penetratedPoints[0];
        penetratedPoint0.y = distanceToRefPlane;
        contactPoints[0] = {refPos + refToWorld * penetratedPoint0, refPos + refToWorld * penetratedPoints[0] };

        float maxDistance = 0;
        int maxDistanceIndex = 0;
        for (int i = 0; i < penetratedPoints.size(); ++i) {
            float distance = glm::distance2(penetratedPoints[0], penetratedPoints[i]);
            if (distance > maxDistance) {
                maxDistance = distance;
                maxDistanceIndex = i;
            }
        }

        penetratedPoint0 = penetratedPoints[maxDistanceIndex];
        penetratedPoint0.y = distanceToRefPlane;
        contactPoints[1] = {
            refPos + refToWorld * penetratedPoint0, refPos + refToWorld * penetratedPoints[maxDistanceIndex]
        };

        float maxArea = 0;
        int maxAreaIndex = 0;
        for (int i = 0; i < penetratedPoints.size(); ++i) {
            auto ca = penetratedPoints[0] - penetratedPoints[i];
            auto cb = penetratedPoints[maxDistanceIndex] - penetratedPoints[i];
            float area = glm::determinant(glm::mat2(glm::vec2(ca[clipX], ca[clipY]), glm::vec2(cb[clipX], cb[clipY])));
            if (area > maxArea) {
                maxArea = area;
                maxAreaIndex = i;
            }
        }

        penetratedPoint0 = penetratedPoints[maxAreaIndex];
        penetratedPoint0.y = distanceToRefPlane;
        contactPoints[2] = {
            refPos + refToWorld * penetratedPoint0, refPos + refToWorld * penetratedPoints[maxAreaIndex]
        };

        float minArea = 0;
        int minAreaIndex = 0;
        for (int i = 0; i < penetratedPoints.size(); ++i) {
            auto da = penetratedPoints[0] - penetratedPoints[i];
            auto db = penetratedPoints[maxDistanceIndex] - penetratedPoints[i];
            float area = glm::determinant(glm::mat2(glm::vec2(da[clipX], da[clipY]), glm::vec2(db[clipX], db[clipY])));
            if (area < minArea) {
                minArea = area;
                minAreaIndex = i;
            }
        }

        penetratedPoint0 = penetratedPoints[minAreaIndex];
        penetratedPoint0.y = distanceToRefPlane;
        contactPoints[3] = {
            refPos + refToWorld * penetratedPoint0, refPos + refToWorld * penetratedPoints[minAreaIndex]
        };

        numPoints = 4;
    } else {
        for (int i = 0; i < penetratedPoints.size(); ++i) {
            glm::vec3 penetratedPoint0 = penetratedPoints[i];
            penetratedPoint0.y = distanceToRefPlane;
            contactPoints[i] = { refPos + refToWorld * penetratedPoint0, refPos + refToWorld * penetratedPoints[i] };
        }
        numPoints = penetratedPoints.size();
    }
}
