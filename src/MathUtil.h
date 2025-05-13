#pragma once

#include <glm/glm.hpp>

namespace physecs {
    inline glm::vec3 multiplyTranspose(const glm::mat3& m, glm::vec3 v) {
        return glm::vec3(glm::dot(v, m[0]), glm::dot(v, m[1]), glm::dot(v, m[2]));
    }

    inline glm::vec3 solve33(const glm::mat3& A, glm::vec3 b) {
        //solve A * x = b
        glm::vec3 cross12 = glm::cross(A[1], A[2]);
        float det = glm::dot(A[0], cross12);
        if (det == 0.f) return glm::vec3(0);
        float invDet = 1.f / det;
        return glm::vec3(
            invDet * glm::dot(b, cross12),
            invDet * glm::dot(A[0], glm::cross(b, A[2])),
            invDet * glm::dot(A[0], glm::cross(A[1], b))
        );
    }
}
