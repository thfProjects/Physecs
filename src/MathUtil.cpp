#include "MathUtil.h"

namespace physecs {

void diagonalizeSymmetric3x3(glm::mat3 m, glm::vec3& outDiagonal, glm::mat3& outRotation) {

    constexpr int numSweeps = 4;

    outRotation = glm::mat3(1.f);

    auto doJacobiRotation = [&](int i, int j) {
        float meanAbsDiag = 0.5f * (glm::abs(m[i][i]) + glm::abs(m[j][j]));
        constexpr float eps = 1e-6;

        if (glm::abs(m[i][j]) <= eps * meanAbsDiag) return;

        float w = (m[j][j] - m[i][i]) / (2.f * m[i][j]);
        float r = glm::sqrt(w * w + 1.f);
        float tan = (w >= 0.f ? 1.f : -1.f) / (glm::abs(w) + r);
        float cos = 1.f / glm::sqrt(1.f + tan * tan);
        float sin = tan * cos;

        int k = 3 - i - j;

        float mij = m[i][j];
        float mik = m[i][k];
        float mjk = m[j][k];

        m[i][j] = m[j][i] = 0.f;
        m[i][k] = m[k][i] = cos * mik - sin * mjk;
        m[j][k] = m[k][j] = cos * mjk + sin * mik;
        m[i][i] -= tan * mij;
        m[j][j] += tan * mij;

        glm::vec3 ri = outRotation[i];
        glm::vec3 rj = outRotation[j];

        outRotation[i] = cos * ri - sin * rj;
        outRotation[j] = sin * ri + cos * rj;
    };

    for (int i = 0; i < numSweeps; i++) {
        doJacobiRotation(0, 1);
        doJacobiRotation(0, 2);
        doJacobiRotation(1, 2);
    }

    outDiagonal = { m[0][0], m[1][1], m[2][2] };
}

}
