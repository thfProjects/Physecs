#pragma once

#include <glm/glm.hpp>
#include "GJK.h"

namespace physecs {

    template<typename Support0, typename Support1>
    class EPA {

        struct EpaPolytopeEdge {
            int i0;
            int i1;
        };

        struct EpaPolytopeFace {
            int indices[3];
            glm::dvec3 normal;
        };

        class EpaPolytope {
            std::vector<GjkVertex> vertices;
            std::vector<EpaPolytopeFace> faces;

            void createFace(int i0, int i1, int i2) {
                glm::dvec3 normal = glm::cross(vertices[i0].pos - vertices[i1].pos, vertices[i2].pos - vertices[i1].pos);
                double nLen = glm::length(normal);
                if (nLen) {
                    normal /= nLen;
                }
                else {
                    printf("EPA error, triangle normal is 0\n");
                }
                faces.push_back({ i0, i1, i2, normal });
            }

            void destroyFace(int faceIndex) {
                faces[faceIndex] = faces.back();
                faces.pop_back();
            }

            double distanceToFace(EpaPolytopeFace& face) {
                return glm::dot(vertices[face.indices[0]].pos, face.normal);
            }

            bool isFaceVisible(EpaPolytopeFace& face, const glm::dvec3 &p) {
                return glm::dot(face.normal, p - vertices[face.indices[0]].pos) > 0;
            }

        public:
            EpaPolytope(GjkSimplex& s) {
                vertices.assign(&s[0], &s[4]);
                faces.reserve(4);
                glm::dvec3 normal = glm::cross(vertices[0].pos - vertices[1].pos, vertices[2].pos - vertices[1].pos);
                if (glm::dot(vertices[3].pos - vertices[0].pos, normal) < 0) {
                    createFace(0, 1, 2);
                    createFace(3, 1, 0);
                    createFace(3, 2, 1);
                    createFace(3, 0, 2);
                }
                else {
                    createFace(0, 2, 1);
                    createFace(3, 2, 0);
                    createFace(3, 1, 2);
                    createFace(3, 0, 1);
                }
            }

            void findClosestFace(float& distance, glm::dvec3& normal, int& faceIndex) {
                distance = std::numeric_limits<double>::max();
                for (int i = 0; i < faces.size(); i++) {
                    auto& face = faces[i];
                    double dist = distanceToFace(face);
                    if (dist < distance) {
                        distance = dist;
                        normal = face.normal;
                        faceIndex = i;
                    }
                }
            }

            void insertVertex(const GjkVertex &vertex) {
                std::vector<EpaPolytopeEdge> looseEdges;
                looseEdges.reserve(3);
                for (int i = faces.size() - 1; i >= 0 ; --i) {
                    auto& face = faces[i];
                    auto& indices = face.indices;

                    if (isFaceVisible(face, vertex.pos)) {
                        EpaPolytopeEdge edges[3] = { { indices[0], indices[1] }, { indices[1], indices[2] }, { indices[2], indices[0] }};
                        bool found[3] = { false, false, false };

                        //check if any of the edges were already added before
                        for (int j = 0; j < 3; ++j) {
                            auto& newEdge = edges[j];
                            for (int k = looseEdges.size() - 1; k >= 0; --k) {
                                auto& oldEdge = looseEdges[k];
                                if (oldEdge.i0 == newEdge.i1 && oldEdge.i1 == newEdge.i0) {
                                    //edge was already added from other triangle
                                    found[j] = true;
                                    looseEdges[k] = looseEdges.back();
                                    looseEdges.pop_back();
                                    break;
                                }
                            }
                        }

                        for (int j = 0; j < 3; ++j) {
                            if (found[j]) continue;
                            looseEdges.push_back(edges[j]);
                        }

                        destroyFace(i);
                    }
                }

                int vertexIndex = vertices.size();
                vertices.push_back(vertex);

                for (auto& edge : looseEdges) {
                    createFace(vertexIndex, edge.i0, edge.i1);
                }
            }

            void getClosestPoints(int faceIndex, glm::vec3& cp0, glm::vec3& cp1) {
                //get barycentric coordinates for closest point on face
                auto& face = faces[faceIndex];
                auto a = vertices[face.indices[0]];
                auto b = vertices[face.indices[1]];
                auto c = vertices[face.indices[2]];

                glm::dvec3 proj = face.normal + distanceToFace(face);

                glm::vec3 v0 = b.pos - a.pos;
                glm::vec3 v1 = c.pos - a.pos;
                glm::vec3 v2 = proj - a.pos;

                double d00 = glm::dot(v0, v0);
                double d01 = glm::dot(v0, v1);
                double d11 = glm::dot(v1, v1);
                double d20 = glm::dot(v2, v0);
                double d21 = glm::dot(v2, v1);

                double denom = d00 * d11 - d01 * d01;

                double u, v, w;

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

                cp0 = u * a.sp0 + v * b.sp0 + w * c.sp0;
                cp1 = u * a.sp1 + v * b.sp1 + w * c.sp1;
            }
        };

    public:
        static glm::vec3 epa(const Support0 &support0, const Support1 &support1, GjkSimplex& s, glm::vec3& cp0, glm::vec3& cp1) {
            EpaPolytope p(s);

            float minDist;
            glm::dvec3 normal;
            int faceIndex;
            for (int i = 0; i < 100; ++i) {
                p.findClosestFace(minDist, normal, faceIndex);
                GjkVertex v = GJK<Support0, Support1>::getPointOnMinkowskiDiff(support0, support1, normal);
                if (glm::dot(v.pos, normal) - minDist < 0.00001f) break;
                p.insertVertex(v);
            }
            p.getClosestPoints(faceIndex, cp0, cp1);
            return normal;
        }
    };
}


