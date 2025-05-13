#include "GeomUtil.h"
#include <glm/gtx/matrix_operation.hpp>

enum Axis { X, Y, Z };

glm::vec3 closestPointOnSegment(glm::vec3 point, glm::vec3 lineOrig, glm::vec3 lineDir, float min, float max) {
    glm::vec3 r = lineOrig - point;
    float t = -glm::dot(r, lineDir);
    t = glm::clamp(t, min, max);
    return lineOrig + t * lineDir;
}

std::pair<glm::vec3, glm::vec3> closestPointsBetweenSegments(glm::vec3 lineOrig0, glm::vec3 dir0, float min0, float max0, glm::vec3 lineOrig1, glm::vec3 dir1, float min1, float max1) {

    glm::vec3 p0, p1;

    float v1v2 = glm::dot(dir0, dir1);
    glm::vec3 r = lineOrig1 - lineOrig0;
    float rv1 = glm::dot(r, dir0);
    float rv2 = glm::dot(r, dir1);

    float t1 = glm::abs(v1v2) > 1.f - 0.0001f ? 0 : (rv1 * v1v2 - rv2) / (1.f - v1v2 * v1v2);
    float t0 = rv1 + t1 * v1v2;

    if (t0 < min0) {
        t0 = min0;
        p0 = lineOrig0 + t0 * dir0;
        t1 = -glm::dot(lineOrig1 - p0, dir1);
    }
    else if (t0 > max0) {
        t0 = max0;
        p0 = lineOrig0 + t0 * dir0;
        t1 = -glm::dot(lineOrig1 - p0, dir1);
    }
    else {
        p0 = lineOrig0 + t0 * dir0;
    }

    if (t1 < min1) {
        t1 = min1;
        p1 = lineOrig1 + t1 * dir1;
        t0 = -glm::dot(lineOrig0 - p1, dir0);
        t0 = glm::clamp(t0, min0, max0);
        p0 = lineOrig0 + t0 * dir0;
    }
    else if (t1 > max1) {
        t1 = max1;
        p1 = lineOrig1 + t1 * dir1;
        t0 = -glm::dot(lineOrig0 - p1, dir0);
        t0 = glm::clamp(t0, min0, max0);
        p0 = lineOrig0 + t0 * dir0;
    }
    else {
        p1 = lineOrig1 + t1 * dir1;
    }

    return { p0, p1 };
}

std::pair<glm::vec3, glm::vec3> closestPointsBetweenSegments(glm::vec3 lineOrig0, glm::vec3 v0, glm::vec3 lineOrig1, glm::vec3 v1) {

    glm::vec3 p0, p1;

    float v0v0 = glm::dot(v0, v0);

    if (v0v0 < 0.0001f) {
        float v1v1 = glm::dot(v1, v1);

        if (v1v1 < 0.0001f) {
            return { lineOrig0, lineOrig1 };
        }

        float t1 = -glm::dot(lineOrig1 - lineOrig0, v1) / v1v1;
        return { lineOrig0, lineOrig1 + t1 * v1 };
    }

    float v1v1 = glm::dot(v1, v1);

    if (v1v1 < 0.0001f) {
        float t0 = -glm::dot(lineOrig0 - lineOrig1, v0) / v0v0;
        return { lineOrig0 + t0 * v0, lineOrig1 };
    }

    float v0v1 = glm::dot(v0, v1);
    glm::vec3 r = lineOrig1 - lineOrig0;
    float rv0 = glm::dot(r, v0);
    float rv1 = glm::dot(r, v1);

    float denom = v0v0 * v1v1 - v0v1 * v0v1;
    float t1 = glm::abs(denom) < 0.0001f ? 0 : (rv0 * v0v1 - rv1 * v0v0) / denom;
    float t0 = (rv0 + t1 * v0v1) / v0v0;

    if (t0 < 0) {
        t0 = 0;
        p0 = lineOrig0 + t0 * v0;
        t1 = -glm::dot(lineOrig1 - p0, v1) / v1v1;
    }
    else if (t0 > 1) {
        t0 = 1;
        p0 = lineOrig0 + t0 * v0;
        t1 = -glm::dot(lineOrig1 - p0, v1) / v1v1;
    }
    else {
        p0 = lineOrig0 + t0 * v0;
    }

    if (t1 < 0) {
        t1 = 0;
        p1 = lineOrig1 + t1 * v1;
        t0 = -glm::dot(lineOrig0 - p1, v0) / v0v0;
        t0 = glm::clamp(t0, 0.f, 1.f);
        p0 = lineOrig0 + t0 * v0;
    }
    else if (t1 > 1) {
        t1 = 1;
        p1 = lineOrig1 + t1 * v1;
        t0 = -glm::dot(lineOrig0 - p1, v0) / v0v0;
        t0 = glm::clamp(t0, 0.f, 1.f);
        p0 = lineOrig0 + t0 * v0;
    }
    else {
        p1 = lineOrig1 + t1 * v1;
    }

    return { p0, p1 };
}

static float caseTwoZeros(glm::vec3 p, int axis, glm::vec3 halfExtents, float& t, glm::vec3& q) {

    float sqrDist = 0;

    t = halfExtents[axis] - p[axis];

    q[axis] = p[axis] + t;

    int axis1 = (axis + 1) % 3;
    int axis2 = (axis + 2) % 3;

    if (p[axis1] > halfExtents[axis1]) {
        float delta = p[axis1] - halfExtents[axis1];
        sqrDist += delta * delta;
        q[axis1] = halfExtents[axis1];
    }
    else if(p[axis1] < -halfExtents[axis1]) {
        float delta = p[axis1] + halfExtents[axis1];
        sqrDist += delta * delta;
        q[axis1] = -halfExtents[axis1];
    }
    else {
        q[axis1] = p[axis1];
    }

    if (p[axis2] > halfExtents[axis2]) {
        float delta = p[axis2] - halfExtents[axis2];
        sqrDist += delta * delta;
        q[axis2] = halfExtents[axis2];
    }
    else if(p[axis2] < -halfExtents[axis2]) {
        float delta = p[axis2] + halfExtents[axis2];
        sqrDist += delta * delta;
        q[axis2] = -halfExtents[axis2];
    }
    else {
        q[axis2] = p[axis2];
    }

    return sqrDist;
}

static float caseOneZero(glm::vec3 p, int zeroAxis, glm::vec3 dir, glm::vec3 halfExtents, float& t, glm::vec3& q) {
    float sqrDist = 0;

    q[zeroAxis] = p[zeroAxis];

    glm::vec3 pMinusExtents = p - halfExtents;

    int x = (zeroAxis + 1) % 3;
    int y = (zeroAxis + 2) % 3;

    float prod0 = dir[y] * pMinusExtents[x];
    float prod1 = dir[x] * pMinusExtents[y];

    if (prod0 >= prod1) {
        //line intersects "x" axis of AABB
        //closest point is on bottom edge of right face of AABB

        q[x] = halfExtents[x];
        float eY = p[y] + halfExtents[y];
        float delta = prod0 - dir[x] * eY;
        //cross product between vector from the closest point on box to line origin and line direction
        if (delta >= 0) {
            //there is no intersection, so compute distance
            sqrDist += delta * delta; //shortest distance from point to line is (d X v) / | d |
            q[y] = -halfExtents[y];
            t = -(pMinusExtents[x] * dir[x] + eY * dir[y]); //dot product
        }
        else {
            //Line intersects box. Distance is zero.
            q[y] = p[y] - prod0 / dir[x]; //proportions of triangles
            t = -pMinusExtents[x] / dir[x]; //p[x] + dir[x] * t = halfExtents[x]
        }
    }
    else {
        // line intersects "y" axis of AABB
        // Closest point is on top edge of left face of AABB

        q[y] = halfExtents[y];
        float eX = p[x] + halfExtents[x];
        float delta = prod1 - dir[y] * eX;
        if (delta >= 0) {
            sqrDist += delta * delta;
            q[x] = -halfExtents[x];
            t = -(eX * dir[x] + pMinusExtents[y] * dir[y]);
        }
        else {
            q[x] = p[x] - prod1 / dir[y];
            t = -pMinusExtents[y] / dir[y];
        }
    }

    //Now, consider the "z" direction
    if (p[zeroAxis] < -halfExtents[zeroAxis]) {
        float delta = p[zeroAxis] + halfExtents[zeroAxis];
        sqrDist += delta * delta;
        q[zeroAxis] = -halfExtents[zeroAxis];
    }
    else if (p[zeroAxis] > halfExtents[zeroAxis]) {
        float delta = p[zeroAxis] - halfExtents[zeroAxis];
        sqrDist += delta * delta;
        q[zeroAxis] = halfExtents[zeroAxis];
    }

    return sqrDist;
}

static float sqrDistLineAABBFace(glm::vec3 p, glm::vec3 dir, glm::vec3 halfExtents, int x, glm::vec3 pMinusExtents, float& t, glm::vec3& q) {
    q = p;
    t = 0;
    float sqrDist = 0;

    int y = (x + 1) % 3;
    int z = (y + 1) % 3;
    glm::vec3 pPlusExtents = p + halfExtents;
    if (dir[x] * pPlusExtents[y] >= dir[y] * pMinusExtents[x]) {
        // region 0, 5, or 4
        if (dir[x] * pPlusExtents[z] >= dir[z] * pMinusExtents[x]) {
            // region 0 - line intersects face
            q[x] = halfExtents[x];
            q[y] -= pMinusExtents[x] * dir[y]/dir[x];
            q[z] -= pMinusExtents[x] * dir[z]/dir[x];
            t = -pMinusExtents[x] / dir[x];
        }
        else {
            // region 4 or 5
            float sqrLen = dir[x] * dir[x] + dir[z] * dir[z];
            float nearestPointY = p[y] - dir[y] * (dir[x] * pMinusExtents[x] + dir[z] * pPlusExtents[z]) / sqrLen;
            //checking y of nearest point on line to box against box height
            if (nearestPointY <= halfExtents[y]) {
                // region 4
                float tmp = p[y] - nearestPointY;
                float delta = dir[x] * pMinusExtents[x] + dir[y] * tmp + dir[z] * pPlusExtents[z];
                t = -delta;
                sqrDist = pMinusExtents[x] * pMinusExtents[x] + tmp * tmp + pPlusExtents[z] * pPlusExtents[z] - delta * delta;
                q[x] = halfExtents[x];
                q[y] = nearestPointY;
                q[z] = -halfExtents[z];
            }
            else {
                // region 5
                float delta = dir[x] * pMinusExtents[x] + dir[y] * pMinusExtents[y] + dir[z] * pPlusExtents[z];
                t = -delta;
                sqrDist = pMinusExtents[x] * pMinusExtents[x] + pMinusExtents[y] + pMinusExtents[y] + pPlusExtents[z] * pPlusExtents[z] - delta * delta;
                q[x] = halfExtents[x];
                q[y] = halfExtents[y];
                q[z] = -halfExtents[z];
            }
        }
    }
    else {
        // region 1, 2 or 3
        if (dir[x] * pPlusExtents[z] >= dir[z] * pMinusExtents[x]) {
            // region 1 or 2
            float sqrLen = dir[x] * dir[x] + dir[y] * dir[y];
            float nearestPointZ = p[z] - dir[z] * (dir[x] * pMinusExtents[x] + dir[y] * pPlusExtents[y]) / sqrLen;
            if (nearestPointZ <= halfExtents[z]) {
                // region 2
                float tmp = p[z] - nearestPointZ;
                float delta = dir[x] * pMinusExtents[x] + dir[y] * pPlusExtents[y] + dir[z] * tmp;
                t = -delta;
                sqrDist = pMinusExtents[x] * pMinusExtents[x] + pPlusExtents[y] * pPlusExtents[y] + tmp * tmp - delta * delta;
                q[x] = halfExtents[x];
                q[y] = -halfExtents[y];
                q[z] = nearestPointZ;
            }
            else {
                // region 1
                float delta = dir[x] * pMinusExtents[x] + dir[y] * pPlusExtents[y] + dir[z] * pMinusExtents[z];
                t = -delta;
                sqrDist = pMinusExtents[x] * pMinusExtents[x] + pPlusExtents[y] * pPlusExtents[y] + pMinusExtents[z] * pMinusExtents[z] - delta * delta;
                q[x] = halfExtents[x];
                q[y] = -halfExtents[y];
                q[z] = halfExtents[z];
            }
        }
        else {
            //Same checks for y and z edges

            float nearestPointY = p[y] - dir[y] * (dir[x] * pMinusExtents[x] + dir[z] * pPlusExtents[z]) / (dir[x] * dir[x] + dir[z] * dir[z]);
            if (nearestPointY >= -halfExtents[y]) {
                // region 4 or 5
                if (nearestPointY <= halfExtents[y]) {
                    // region 4
                    float tmp = p[y] - nearestPointY;
                    float delta = dir[x] * pMinusExtents[x] + dir[y] * tmp + dir[z] * pPlusExtents[z];
                    t = -delta;
                    sqrDist = pMinusExtents[x] * pMinusExtents[x] + tmp * tmp + pPlusExtents[z] * pPlusExtents[z] - delta * delta;
                    q[x] = halfExtents[x];
                    q[y] = nearestPointY;
                    q[z] = -halfExtents[z];
                }
                else {
                    // region 5
                    float delta = dir[x] * pMinusExtents[x] + dir[y] * pMinusExtents[y] + dir[z] * pPlusExtents[z];
                    t = -delta;
                    sqrDist = pMinusExtents[x] * pMinusExtents[x] + pMinusExtents[y] + pMinusExtents[y] + pPlusExtents[z] * pPlusExtents[z] - delta * delta;
                    q[x] = halfExtents[x];
                    q[y] = halfExtents[y];
                    q[z] = -halfExtents[z];
                }
                return sqrDist;
            }

            float nearestPointZ = p[z] - dir[z] * (dir[x] * pMinusExtents[x] + dir[y] * pPlusExtents[y]) / (dir[x] * dir[x] + dir[y] * dir[y]);
            if (nearestPointZ >= -halfExtents[z]) {
                // region 1 or 2
                if (nearestPointZ <= halfExtents[z]) {
                    // region 2
                    float tmp = p[z] - nearestPointZ;
                    float delta = dir[x] * pMinusExtents[x] + dir[y] * pPlusExtents[y] + dir[z] * tmp;
                    t = -delta;
                    sqrDist = pMinusExtents[x] * pMinusExtents[x] + pPlusExtents[y] * pPlusExtents[y] + tmp * tmp - delta * delta;
                    q[x] = halfExtents[x];
                    q[y] = -halfExtents[y];
                    q[z] = nearestPointZ;
                }
                else {
                    // region 1
                    float delta = dir[x] * pMinusExtents[x] + dir[y] * pPlusExtents[y] + dir[z] * pMinusExtents[z];
                    t = -delta;
                    sqrDist = pMinusExtents[x] * pMinusExtents[x] + pPlusExtents[y] * pPlusExtents[y] + pMinusExtents[z] * pMinusExtents[z] - delta * delta;
                    q[x] = halfExtents[x];
                    q[y] = -halfExtents[y];
                    q[z] = halfExtents[z];
                }
                return sqrDist;
            }

            // region 3
            float delta = dir[x] * pMinusExtents[x] + dir[y] * pPlusExtents[y] + dir[z] * pPlusExtents[z];
            t = -delta;
            sqrDist = pMinusExtents[x] * pMinusExtents[x] + pPlusExtents[y] * pPlusExtents[y] + pPlusExtents[z] * pPlusExtents[z] - delta * delta;
            q[x] = halfExtents[x];
            q[y] = -halfExtents[y];
            q[z] = -halfExtents[z];
        }
    }

    return sqrDist;
}

static float caseNoZeroes(glm::vec3 p, glm::vec3 dir, glm::vec3 halfExtents, float& t, glm::vec3& q) {
    float sqrDist = 0;

    glm::vec3 pMinusExtents = p - halfExtents;
    float dxEy = dir.x * pMinusExtents.y;
    float dyEx = dir.y * pMinusExtents.x;
    if (dyEx >= dxEy) {
        float dzEx = dir.z * pMinusExtents.x;
        float dxEz = dir.x * pMinusExtents.z;
        if (dzEx >= dxEz) {
            sqrDist = sqrDistLineAABBFace(p, dir, halfExtents, X, pMinusExtents, t, q);
        }
        else {
            sqrDist = sqrDistLineAABBFace(p, dir, halfExtents, Z, pMinusExtents, t, q);
        }
    }
    else {
        float dzEy = dir.z * pMinusExtents.y;
        float dyEz = dir.y * pMinusExtents.z;
        if (dzEy >= dyEz) {
            sqrDist = sqrDistLineAABBFace(p, dir, halfExtents, Y, pMinusExtents, t, q);
        }
        else {
            sqrDist = sqrDistLineAABBFace(p, dir, halfExtents, Z, pMinusExtents, t, q);
        }
    }
    return sqrDist;
}

static float sqrDistPointAABB(glm::vec3 p, glm::vec3 halfExtents, glm::vec3& q) {
    q = glm::clamp(p, -halfExtents, halfExtents);
    glm::vec3 qToP = p - q;
    return glm::dot(qToP, qToP);
}

static float sqrDistLineAABB(glm::vec3 p, glm::vec3 dir, glm::vec3 halfExtents, float& t, glm::vec3& q) {
    bool reflect[3];
    for(int i=0;i<3;i++) {
        if(dir[i]<0.0f) {
            p[i] = -p[i];
            dir[i] = -dir[i];
            reflect[i] = true;
        }
        else {
            reflect[i] = false;
        }
    }

    float sqrDist = 0;

    if (dir.x > 0) {
        if (dir.y > 0) {
            if (dir.z > 0) sqrDist = caseNoZeroes(p, dir, halfExtents, t, q);
            else sqrDist = caseOneZero(p, Z, dir, halfExtents, t, q);
        }
        else {
            if (dir.z > 0) sqrDist = caseOneZero(p, Y, dir, halfExtents, t, q);
            else sqrDist = caseTwoZeros(p, X, halfExtents, t, q);
        }
    }
    else {
        if (dir.y > 0) {
            if (dir.z > 0) sqrDist = caseOneZero(p, X, dir, halfExtents, t, q);
            else sqrDist = caseTwoZeros(p, Y, halfExtents, t, q);
        }
        else {
            if (dir.z > 0) sqrDist = caseTwoZeros(p, Z, halfExtents, t, q);
            else {
                sqrDist = sqrDistPointAABB(p, halfExtents, q);
                t = 0;
            }
        }
    }

    for(int i=0;i<3;i++) {
        if (reflect[i]) {
            q[i] = -q[i];
        }
    }

    return sqrDist;
}

float sqrDistSegmentAABB(glm::vec3 p, glm::vec3 dir, float min, float max, glm::vec3 halfExtents, float& t, glm::vec3& q) {
    float sqrDist = sqrDistLineAABB(p, dir, halfExtents, t, q);
    if (t < min) {
        t = min;
        sqrDist = sqrDistPointAABB(p + dir * t, halfExtents, q);
    }
    else if (t > max) {
        t = max;
        sqrDist = sqrDistPointAABB(p + dir * t, halfExtents, q);
    }

    return sqrDist;
}

float sqrDistPointTriangle(glm::vec3 p, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3& q, TriangleFeature &triangleFeature, char &featureIndex) {
    glm::vec3 ab = b - a;
    glm::vec3 ac = c - a;

    glm::vec3 ap = p - a;

    float d1 = glm::dot(ab, ap);
    float d2 = glm::dot(ac, ap);

    if (d1 <= 0.f && d2 <= 0.f) {
        triangleFeature = TriangleFeature::VERTEX;
        featureIndex = 0;
        q = a;
        goto end;
    }

    glm::vec3 bp = p - b;
    float d3 = glm::dot(ab, bp);
    float d4 = glm::dot(ac, bp);

    if (d3 >= 0.f && d4 <= d3) {
        triangleFeature = TriangleFeature::VERTEX;
        featureIndex = 1;
        q = b;
        goto end;
    }

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0.f && d3 <= 0.f) {
        triangleFeature = TriangleFeature::EDGE;
        featureIndex = 2;
        q = a + d1 / (d1 - d3) * ab;
        goto end;
    }

    glm::vec3 cp = p - c;
    float d5 = glm::dot(ab, cp);
    float d6 = glm::dot(ac, cp);

    if (d6 >= 0.f && d5 <= d6) {
        triangleFeature = TriangleFeature::VERTEX;
        featureIndex = 2;
        q = c;
        goto end;
    }

    float va = d3 * d6 - d5 * d4;
    if (va <= 0 && d4 >= d3 && d5 >= d6) {
        triangleFeature = TriangleFeature::EDGE;
        featureIndex = 0;
        q = b + (d4 - d3) / (d4 - d3 + d5 - d6) * (c - b);
        goto end;
    }

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0.f && d6 <= 0.f) {
        triangleFeature = TriangleFeature::EDGE;
        featureIndex = 1;
        q = c + d2 / (d2 - d6) * ac;
        goto end;
    }

    //p must project on face
    triangleFeature = TriangleFeature::FACE;
    float denom = va + vb + vc;
    float u = va / denom;
    float v = vb / denom;
    float w = 1 - u - v;
    q = u * a + v * b + w * c;

end:
    glm::vec3 d = q - p;
    return glm::dot(d, d);
}

static float sqrDistLineSegment(glm::vec3 p, glm::vec3 dir, glm::vec3 a, glm::vec3 b, float& t, glm::vec3& q, char& vertexFlag) {
    glm::vec3 ab = b - a;
    float v1v2 = glm::dot(dir, ab);
    float v2v2 = glm::dot(ab, ab);
    glm::vec3 r = p - a;
    float rv1 = glm::dot(r, dir);
    float rv2 = glm::dot(r, ab);
    float denom = (v1v2 * v1v2 - v2v2);

    if (glm::abs(denom) <= 1e-6f) {
        //lines are parallel
        q = a;
        t = -rv1;
        vertexFlag = 0;
        glm::vec3 d = p + t * dir - a;
        return glm::dot(d, d);
    }

    float s = (rv1 * v1v2 - rv2) / denom;

    if (s <= 0.f) {
        q = a;
        t = -rv1;
        vertexFlag = 1;
        glm::vec3 d = p + t * dir - a;
        return glm::dot(d, d);
    }

    if (s >= 1.f) {
        q = b;
        t = -glm::dot(p - b, dir);
        vertexFlag = 2;
        glm::vec3 d = p + t * dir - b;
        return glm::dot(d, d);
    }

    q = a + s * ab;
    t = s * v1v2 - rv1;
    vertexFlag = 0;
    glm::vec3 d = p + t * dir - q;
    return glm::dot(d, d);
}

static float sqrDistLineTriangle(glm::vec3 p, glm::vec3 dir, glm::vec3 a, glm::vec3 b, glm::vec3 c, float &t, glm::vec3& q, TriangleFeature &triangleFeature, char &featureIndex) {
    glm::vec3 e0 = b - a;
    glm::vec3 e1 = c - a;

    glm::mat3 A = glm::mat3(e0, e1, -dir);

    float det = glm::determinant(A);

    if (glm::abs(det) > 1e-6f) {
        //line is not parallel to triangle
        glm::vec3 diff = p - a;
        float invDet = 1 / det;
        float u = invDet * glm::determinant(glm::mat3(diff, e1, -dir));
        float v = invDet * glm::determinant(glm::mat3(e0, diff, -dir));
        float w = 1 - u - v;

        if (u >= 0.f && v >= 0.f && w >= 0.f) {
            //line intersects triangle
            t = invDet * glm::determinant(glm::mat3(e0, e1, diff));
            q = a + u * e0 + v * e1;
            triangleFeature = TriangleFeature::FACE;
            return 0;
        }
    }

    //line is parallel to triangle or is not parallel and doesn't intersect
    //find closest edge to line

    float t1;
    glm::vec3 q1;
    char vertexFlag, vertexFlag1;

    featureIndex = 2;

    float dist = sqrDistLineSegment(p, dir, a, b, t, q, vertexFlag); //ab
    float dist1 = sqrDistLineSegment(p, dir, b, c, t1, q1, vertexFlag1); //bc

    if (dist1 < dist) {
        featureIndex = 0;
        t = t1;
        q = q1;
        vertexFlag = vertexFlag1;
        dist = dist1;
    }

    dist1 = sqrDistLineSegment(p, dir, c, a, t1, q1, vertexFlag1); //ac

    if (dist1 < dist) {
        featureIndex = 1;
        t = t1;
        q = q1;
        vertexFlag = vertexFlag1;
        dist = dist1;
    }

    if (vertexFlag) {
        triangleFeature = TriangleFeature::VERTEX;
        featureIndex = (featureIndex + vertexFlag) % 3;
    }
    else {
        triangleFeature = TriangleFeature::EDGE;
    }

    return dist;
}

float sqrDistSegmentTriangle(glm::vec3 p, glm::vec3 dir, float min, float max, glm::vec3 a, glm::vec3 b, glm::vec3 c, float& t, glm::vec3& q, TriangleFeature& triangleFeature, char& featureIndex) {
    float sqrDist = sqrDistLineTriangle(p, dir, a, b, c, t, q, triangleFeature, featureIndex);
    if (t < min) {
        t = min;
        sqrDist = sqrDistPointTriangle(p + dir * t, a, b, c, q, triangleFeature, featureIndex);
    }
    else if (t > max) {
        t = max;
        sqrDist = sqrDistPointTriangle(p + dir * t, a, b, c, q, triangleFeature, featureIndex);
    }
    return sqrDist;
}

float distanceAABBPlane(glm::vec3 halfExtents, glm::vec3 n, float d) {
    float r = halfExtents.x * glm::abs(n.x) + halfExtents.y * glm::abs(n.y) + halfExtents.z * glm::abs(n.z);
    return glm::abs(d) - r;
}
