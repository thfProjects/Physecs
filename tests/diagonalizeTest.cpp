#include <MathUtil.h>
#include <stdio.h>
#include <glm/glm.hpp>
#include <random>
#include <glm/gtx/matrix_operation.hpp>

glm::mat3 generateRandomSymmetric3x3(float scale, std::default_random_engine& engine) {
    std::uniform_real_distribution uniform_dist(-1.f, 1.f);

    glm::mat3 mat = glm::diagonal3x3(scale * glm::vec3{ uniform_dist(engine), uniform_dist(engine), uniform_dist(engine) });
    for (int i = 0; i < 3; ++i) {
        for (int j = i + 1; j < 3; ++j) {
            mat[i][j] = mat[j][i] = uniform_dist(engine) * scale;
        }
    }

    return mat;
}

bool checkRandomSymmetric3x3(float scale, std::default_random_engine& engine) {
    const float tolerance = 1e-4 * scale;

    glm::mat3 mat = generateRandomSymmetric3x3(scale, engine);

    glm::vec3 diag;
    glm::mat3 rot;
    physecs::diagonalizeSymmetric3x3(mat, diag, rot);

    glm::mat3 matRecovered = rot * glm::diagonal3x3(diag) * glm::transpose(rot);

    bool passed = true;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (glm::distance(mat[i][j], matRecovered[i][j]) > tolerance) {
                passed = false;
                printf("Error checkRandomSymmetric scale %f: different values in original and recovered matrix at position %d, %d: %f, %f\n", scale, i, j, mat[i][j], matRecovered[i][j]);
            }
        }
    }

    return passed;
}

int main() {
    std::default_random_engine engine;

    int numFailed = 0;

    constexpr int numIterations = 1000;

    for (int i = 0; i < numIterations; ++i) {
        numFailed += !checkRandomSymmetric3x3(1.f, engine);
        numFailed += !checkRandomSymmetric3x3(1000.f, engine);
        numFailed += !checkRandomSymmetric3x3(1e-4f, engine);
    }

    printf("Test finished with %d errors\n", numFailed);

    return numFailed ? 1 : 0;
}
