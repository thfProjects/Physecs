#pragma once

#include <Profiling.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "ConvexMesh.h"
#include "MathUtil.h"
#include "glm/gtx/norm.hpp"

namespace physecs {

    struct SphereSupportFunction {
        glm::vec3 pos;
        float radius;

        glm::vec3 operator() (glm::vec3 dir) const {
            return pos + dir * radius;
        }
    };

    struct CapsuleSupportFunction {
        glm::vec3 pos;
        glm::vec3 axis;
        float halfHeight;
        float radius;

        glm::vec3 operator() (glm::vec3 dir) const {
            return pos + axis * glm::sign(glm::dot(dir, axis)) * halfHeight + dir * radius;
        }
    };

    struct BoxSupportFunction {
        glm::vec3 pos;
        glm::mat3 localToWorld;
        glm::vec3 halfExtents;

        glm::vec3 operator() (glm::vec3 dir) const {
            glm::vec3 support = pos;
            for (int i = 0; i < 3; ++i) {
                support += glm::sign(glm::dot(dir, localToWorld[i])) * localToWorld[i] * halfExtents[i];
            }
            return support;
        }
    };

    struct ConvexMeshSupportFunction {
        glm::vec3 pos;
        glm::mat3 localToWorld;
        ConvexMeshVertices<4>& vertices;
        glm::vec3 scale;

        glm::vec3 operator() (glm::vec3 dir) const {
            dir = scale * multiplyTranspose(localToWorld, dir);

            const auto increment = _mm_set1_epi32(4);
            auto indices = _mm_setr_epi32(0, 1, 2, 3);
            auto maxIndices = indices;
            auto maxValues = _mm_set1_ps(std::numeric_limits<float>::lowest());

            auto dirX = _mm_set1_ps(dir.x);
            auto dirY = _mm_set1_ps(dir.y);
            auto dirZ = _mm_set1_ps(dir.z);
            for (int i = 0; i < vertices.size(); i += 4) {
                const float* f = glm::value_ptr(vertices[i]);

                const auto r0 = _mm_loadu_ps(f);
                const auto r1 = _mm_loadu_ps(f + 4);
                const auto r2 = _mm_loadu_ps(f + 8);

                const auto xy = _mm_shuffle_ps(r1, r2, _MM_SHUFFLE(2, 1, 3, 2));
                const auto yz = _mm_shuffle_ps(r0, r1, _MM_SHUFFLE(1, 0, 2, 1));

                auto x = _mm_shuffle_ps(r0, xy, _MM_SHUFFLE(2, 0, 3, 0));
                auto y = _mm_shuffle_ps(yz, xy, _MM_SHUFFLE(3, 1, 2, 0));
                auto z = _mm_shuffle_ps(yz, r2, _MM_SHUFFLE(3, 0, 3, 1));

                x = _mm_mul_ps(x, dirX);
                y = _mm_mul_ps(y, dirY);
                z = _mm_mul_ps(z, dirZ);

                auto dot = _mm_add_ps(x, y);
                dot = _mm_add_ps(dot, z);

                auto gt = _mm_cmpgt_ps(dot, maxValues);
                maxIndices = _mm_blendv_epi8(maxIndices, indices, _mm_castps_si128(gt));
                maxValues = _mm_max_ps(dot, maxValues);

                indices = _mm_add_epi32(indices, increment);
            }

            float valuesArr[4];
            int indicesArr[4];

            _mm_storeu_ps(valuesArr, maxValues);
            _mm_storeu_epi32(indicesArr, maxIndices);

            int maxIndex = indicesArr[0];
            float maxValue = valuesArr[0];
            for (int i = 1; i < 4; ++i) {
                if (valuesArr[i] > maxValue) {
                    maxValue = valuesArr[i];
                    maxIndex = indicesArr[i];
                }
            }

            return pos + localToWorld * (scale * vertices[maxIndex]);
        }
    };

    struct TriangleSupportFunction {
        glm::vec3 a;
        glm::vec3 b;
        glm::vec3 c;

        glm::vec3 operator() (glm::vec3 dir) const {
            if (glm::dot(dir, a) > glm::dot(dir, b)) {
                if (glm::dot(dir, a) > glm::dot(dir, c)) {
                    return a;
                }
                return c;
            }
            if (glm::dot(dir, b) > glm::dot(dir, c)) {
                return b;
            }
            return c;
        }
    };

    struct GjkVertex {
        glm::dvec3 pos;
        glm::dvec3 sp0;
        glm::dvec3 sp1;
    };

    struct GjkSimplex {
        GjkVertex v[4];

        GjkVertex& operator[](int i) { return v[i]; }
    };

    template <typename Support0, typename  Support1>
    class GJK {
        static float sqrDistPointToLine(glm::vec3 p, glm::vec3 a, glm::vec3 b) {
            glm::vec3 r = p - a;
            glm::vec3 d = b - a;
            glm::vec3 q = a + glm::dot(r, d) / glm::dot(d, d) * d;
            glm::vec3 pq = p - q;
            return glm::dot(pq, pq);
        }

        static float distPointToPlane(glm::vec3 p, glm::vec3 o, glm::vec3 n) {
            return glm::abs(glm::dot(p - o, n));
        }

        static bool isPastOrigin(const glm::dvec3 &p, const glm::dvec3 &dir) {
            return glm::dot(p, dir) >= 0;
        }

        static bool isZero(glm::vec3 v) {
            return glm::length2(v) < 1e-6;
        }

        static bool checkDirection(const Support0 &support0, const Support1 &support1, glm::dvec3 dir, GjkVertex& v) {
            dir = glm::normalize(dir);
            v = getPointOnMinkowskiDiff(support0, support1, dir);
            if (!isPastOrigin(v.pos, dir)) return false;
            return true;
        }

        static bool checkFace(const glm::dvec3 &v0, const glm::dvec3 &v1, const glm::dvec3 &v2, const glm::dvec3 &opposite, glm::dvec3& n) {
            n = glm::cross(v1 - v0, v2 - v0);
            n = glm::sign(glm::dot(v0 - opposite, n)) * n;
            return glm::dot(n, -v0) > 0.00001f;
        }

        static bool simplexContainsOrigin(GjkSimplex& s, glm::dvec3& dir, int& faceIndex) {
            faceIndex = 2;
            if (checkFace(s[3].pos, s[0].pos, s[1].pos, s[faceIndex].pos, dir)) return false;
            if (checkFace(s[3].pos, s[2].pos, s[0].pos, s[--faceIndex].pos, dir)) return false;
            if (checkFace(s[3].pos, s[1].pos, s[2].pos, s[--faceIndex].pos, dir)) return false;
            return true;
        }
    public:
        static GjkVertex getPointOnMinkowskiDiff(const Support0 &support0, const Support1 &support1, glm::vec3 dir) {
            glm::vec3 sp0 = support0(dir);
            glm::vec3 sp1 = support1(-dir);
            return { sp0 - sp1, sp0, sp1 };
        }

        static bool gjk(const Support0 &support0, const Support1 &support1, glm::vec3 startDir) {
            GjkSimplex s;

            //get initial point
            glm::dvec3 dir = glm::normalize(startDir);
            s[0] = getPointOnMinkowskiDiff(support0, support1, dir);

            //get the furthest point towards origin
            dir = -s[0].pos;
            if (isZero(dir)) return true;
            if (!checkDirection(support0, support1, dir, s[1])) return false;

            //get the furthest point towards origin along normal to line v0v1
            dir = glm::cross(glm::cross(s[0].pos - s[1].pos, -s[1].pos), s[0].pos - s[1].pos);
            if (isZero(dir)) return true;
            if (!checkDirection(support0, support1, dir, s[2])) return false;

            //get the furthest point towards origin along normal to face v0, v1, v2
            dir = glm::cross(s[1].pos - s[0].pos, s[2].pos - s[0].pos);
            dir = glm::sign(glm::dot(-s[0].pos, dir)) * dir;
            if (isZero(dir)) return true;
            if (!checkDirection(support0, support1, dir, s[3])) return false;

            int faceIndex;
            for (int i = 0; i < 100; ++i) {
                if (simplexContainsOrigin(s, dir, faceIndex)) return true;
                if (!checkDirection(support0, support1, dir, s[faceIndex])) return false;
                auto temp = s[faceIndex];
                s[faceIndex] = s[3];
                s[3] = temp;
            }

            return false;
        }

        static bool gjk(const Support0 &support0, const Support1 &support1, glm::vec3 startDir, GjkSimplex& s) {
            //get initial point
            glm::dvec3 dir = glm::normalize(startDir);
            s[0] = getPointOnMinkowskiDiff(support0, support1, dir);

            //get the furthest point towards origin
            if (isZero(s[0].pos)) {
                //v0 is origin, build arbitrary simplex with it
                dir = -dir;
                s[1] = getPointOnMinkowskiDiff(support0, support1, dir);

                dir = glm::cross(s[0].pos - s[1].pos, glm::dvec3(0, 1, 0));
                if(isZero(dir)) dir = glm::cross(s[0].pos - s[1].pos, glm::dvec3(0, 0, 1));
                s[2] = getPointOnMinkowskiDiff(support0, support1, glm::normalize(dir));
                if (sqrDistPointToLine(s[2].pos, s[0].pos, s[1].pos) < 0.00001f) s[2] = getPointOnMinkowskiDiff(support0, support1, -dir);

                dir = glm::normalize(glm::cross(s[1].pos - s[0].pos, s[2].pos - s[0].pos));
                s[3] = getPointOnMinkowskiDiff(support0, support1, dir);
                if (distPointToPlane(s[3].pos, s[0].pos, dir) < 0.0001f) s[3] = getPointOnMinkowskiDiff(support0, support1, -dir);

                return true;
            }
            dir = -s[0].pos;
            if (!checkDirection(support0, support1, dir, s[1])) return false;

            //get the furthest point towards origin along normal to line v0v1
            dir = glm::cross(glm::cross(s[0].pos - s[1].pos, -s[1].pos), s[0].pos - s[1].pos);
            if (isZero(dir)) {
                //origin is on line v0v1
                dir = glm::cross(s[0].pos - s[1].pos, glm::dvec3(0, 1, 0));
                if(isZero(dir)) dir = glm::cross(s[0].pos - s[1].pos, glm::dvec3(0, 0, 1));
                s[2] = getPointOnMinkowskiDiff(support0, support1, glm::normalize(dir));
                if (sqrDistPointToLine(s[2].pos, s[0].pos, s[1].pos) < 0.00001f) s[2] = getPointOnMinkowskiDiff(support0, support1, -dir);

                dir = glm::normalize(glm::cross(s[1].pos - s[0].pos, s[2].pos - s[0].pos));
                s[3] = getPointOnMinkowskiDiff(support0, support1, dir);
                if (distPointToPlane(s[3].pos, s[0].pos, dir) < 0.0001f) s[3] = getPointOnMinkowskiDiff(support0, support1, -dir);

                return true;
            }
            if (!checkDirection(support0, support1, dir, s[2])) return false;

            //get the furthest point towards origin along normal to face v0, v1, v2
            dir = glm::cross(s[1].pos - s[0].pos, s[2].pos - s[0].pos);
            auto sign = glm::sign(glm::dot(-s[0].pos, dir));
            if (!sign) {
                //origin is on triangle v0, v1, v2
                dir = glm::normalize(dir);
                s[3] = getPointOnMinkowskiDiff(support0, support1, dir);
                if (distPointToPlane(s[3].pos, s[0].pos, dir) < 0.0001f) s[3] = getPointOnMinkowskiDiff(support0, support1, -dir);

                return true;
            }
            dir = sign * dir;
            if (!checkDirection(support0, support1, dir, s[3])) return false;

            int faceIndex;
            for (int i = 0; i < 100; ++i) {
                if (simplexContainsOrigin(s, dir, faceIndex)) return true;
                if (!checkDirection(support0, support1, dir, s[faceIndex])) return false;
                auto temp = s[faceIndex];
                s[faceIndex] = s[3];
                s[3] = temp;
            }

            return false;
        }
    };
}




