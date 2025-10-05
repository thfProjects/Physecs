#include "Constraint1DContainer.h"

#include <bitset>

#include "Transform.h"
#include <Constraint1D.h>
#include <iostream>

#include "SIMD.h"

#define INT32(c) ((int)(((unsigned char)c) << 24) |  (int)(((unsigned char)c) << 16) |  (int)(((unsigned char)c) << 8) |  (int)((unsigned char)c))

enum flagsW : int {
    SOFT_W = 1 | 1 << 8 | 1 << 16 | 1 << 24,
    ANGULAR_W = 1 << 1 | 1 << 9 | 1 << 17 | 1 << 25,
    LIMITED_W = 1 << 2 | 1 << 10 | 1 << 18 | 1 << 26,
};

inline void fill(float* data, unsigned int size, float value) {
    const auto block = _mm_set1_ps(value);
    for (unsigned int i = 0; i < size; i += 4) {
        _mm_store_ps(data + i, block);
    }
}

inline void fill(unsigned char* data, unsigned int size, unsigned char value) {
    const auto block = _mm_set1_epi8(value);
    const unsigned int simdSize = size / 16 * 16;
    for (unsigned int i = 0; i < simdSize; i += 16) {
        _mm_store_si128(reinterpret_cast<__m128i*>(data + i), block);
    }
    for (unsigned int i = simdSize; i < size; ++i) {
        data[i] = value;
    }
}

inline void fill(glm::vec3* data, unsigned int size, float value) {
    const auto block = _mm_set1_ps(value);
    for (unsigned int i = 0; i < size; i += 4) {
        float* f = glm::value_ptr(data[i]);
        _mm_store_ps(f, block);
        _mm_store_ps(f + 4, block);
        _mm_store_ps(f + 8, block);
    }
}

void physecs::Constraint1DSoa::pushBack(TransformComponent &transform0, TransformComponent &transform1, RigidBodyDynamicComponent *dynamic0, RigidBodyDynamicComponent *dynamic1) {
    if (size == capacity) {
        const int newCapacity = capacity ? 2 * capacity : 4;

        transformComponentsBuffer.reallocate(capacity, newCapacity);
        dynamicComponentsBuffer.reallocate(capacity, newCapacity);
        linearBuffer.reallocate(capacity, newCapacity);
        angular0Buffer.reallocate(capacity, newCapacity);
        angular1Buffer.reallocate(capacity, newCapacity);
        targetVelocityBuffer.reallocate(capacity, newCapacity);
        cBuffer.reallocate(capacity, newCapacity);
        minBuffer.reallocate(capacity, newCapacity);
        maxBuffer.reallocate(capacity, newCapacity);
        flagsBuffer.reallocate(capacity, newCapacity);
        frequencyBuffer.reallocate(capacity, newCapacity);
        dampingRatioBuffer.reallocate(capacity, newCapacity);
        angular0tBuffer.reallocate(capacity, newCapacity);
        angular1tBuffer.reallocate(capacity, newCapacity);
        invEffMassBuffer.reallocate(capacity, newCapacity);
        totalLambdaBuffer.reallocate(capacity, newCapacity);

        velocity0Buffer.reallocate(capacity, newCapacity);
        velocity1Buffer.reallocate(capacity, newCapacity);
        angularVelocity0Buffer.reallocate(capacity, newCapacity);
        angularVelocity1Buffer.reallocate(capacity, newCapacity);
        invMass0Buffer.reallocate(capacity, newCapacity);
        invMass1Buffer.reallocate(capacity, newCapacity);
        position0Buffer.reallocate(capacity, newCapacity);
        position1Buffer.reallocate(capacity, newCapacity);
        orientation0Buffer.reallocate(capacity, newCapacity);
        orientation1Buffer.reallocate(capacity, newCapacity);

        capacity = newCapacity;
    }

    transformComponentsBuffer[size] = { &transform0, &transform1 };
    dynamicComponentsBuffer[size] = { dynamic0, dynamic1 };

    ++size;
}

void physecs::Constraint1DSoa::fillDefaults() {
    fill(linearBuffer.get(), size, 0);
    fill(angular0Buffer.get(), size, 0);
    fill(angular1Buffer.get(), size, 0);
    fill(targetVelocityBuffer.get(), size, 0);
    fill(cBuffer.get(), size, 0);
    fill(minBuffer.get(), size, std::numeric_limits<float>::lowest());
    fill(maxBuffer.get(), size, std::numeric_limits<float>::max());
    fill(flagsBuffer.get(), size, 0);
    fill(frequencyBuffer.get(), size, 0);
    fill(dampingRatioBuffer.get(), size, 0);
    fill(invEffMassBuffer.get(), size, 0);
    fill(totalLambdaBuffer.get(), size, 0);

    fill(velocity0Buffer.get(), size, 0);
    fill(velocity1Buffer.get(), size, 0);
    fill(angularVelocity0Buffer.get(), size, 0);
    fill(angularVelocity1Buffer.get(), size, 0);
    fill(invMass0Buffer.get(), size, 0);
    fill(invMass0Buffer.get(), size, 0);
}

void physecs::Constraint1DSoa::clear() {
    size = 0;
}

void physecs::Constraint1DSoa::preSolve() {
    for (int i = 0; i < size; i++) {
        auto [dynamic0, dynamic1] = dynamicComponentsBuffer[i];
        auto [transform0, transform1] = transformComponentsBuffer[i];
        auto& linear = linearBuffer[i];
        auto& angular0 = angular0Buffer[i];
        auto& angular1 = angular1Buffer[i];
        auto& angular0t = angular0tBuffer[i];
        auto& angular1t = angular1tBuffer[i];
        const auto& c = cBuffer[i];
        const auto& flags = flagsBuffer[i];
        const auto& min = minBuffer[i];
        const auto& max = maxBuffer[i];
        auto& invEffMass = invEffMassBuffer[i];
        auto& totalLambda = totalLambdaBuffer[i];

        auto& invMass0 = invMass0Buffer[i];
        auto& invMass1 = invMass1Buffer[i];

        glm::mat3 invInertiaTensor0(0);
        if (dynamic0 && !dynamic0->isKinematic) {
            invMass0 = dynamic0->invMass;
            invInertiaTensor0 = dynamic0->invInertiaTensorWorld;
        }

        glm::mat3 invInertiaTensor1(0);
        if (dynamic1 && !dynamic1->isKinematic) {
            invMass1 = dynamic1->invMass;
            invInertiaTensor1 = dynamic1->invInertiaTensorWorld;
        }

        angular0t = invInertiaTensor0 * angular0;
        angular1t = invInertiaTensor1 * angular1;

        invEffMass = glm::dot(angular0, angular0t) + glm::dot(angular1, angular1t);
        if (!(flags & Constraint1D::ANGULAR)) {
            invEffMass += glm::dot(linear, linear) * (invMass0 + invMass1);
        }

        // correct position error
        if (!(flags & Constraint1D::SOFT) && c && invEffMass) {
            float lambda = 0.1f * c / invEffMass;
            if (flags & Constraint1D::LIMITED)
                lambda = glm::clamp(lambda, min, max);

            if (!(flags & Constraint1D::ANGULAR)) {
                transform0->position += lambda * invMass0 * linear;
                transform1->position -= lambda * invMass1 * linear;
            }

            transform0->orientation += 0.5f * glm::quat(0, lambda * angular0t) * transform0->orientation;
            transform0->orientation = glm::normalize(transform0->orientation);

            transform1->orientation -= 0.5f * glm::quat(0, lambda * angular1t) * transform1->orientation;
            transform1->orientation = glm::normalize(transform1->orientation);
        }

        // warm start
        if (flags & Constraint1D::SOFT || glm::abs(c) > 1e-4 || glm::abs(totalLambda) > 10000) continue;

        totalLambda = totalLambda * 0.5f;
        if (dynamic0 && !dynamic0->isKinematic) {
            if (!(flags & Constraint1D::ANGULAR))
                dynamic0->velocity += totalLambda * dynamic0->invMass * linear;
            dynamic0->angularVelocity += totalLambda * angular0t;
        }

        if (dynamic1 && !dynamic1->isKinematic) {
            if (!(flags & Constraint1D::ANGULAR))
                dynamic1->velocity -= totalLambda * dynamic1->invMass * linear;
            dynamic1->angularVelocity -= totalLambda * angular1t;
        }
    }
}

void physecs::Constraint1DSoa::preSolveSimd() {
    for (int i = 0; i < size; i++) {
        auto [dynamic0, dynamic1] = dynamicComponentsBuffer[i];
        auto [transform0, transform1] = transformComponentsBuffer[i];
        auto& linear = linearBuffer[i];
        auto& angular0 = angular0Buffer[i];
        auto& angular1 = angular1Buffer[i];
        auto& angular0t = angular0tBuffer[i];
        auto& angular1t = angular1tBuffer[i];
        const auto& flags = flagsBuffer[i];
        auto& invEffMass = invEffMassBuffer[i];
        auto& invMass0 = invMass0Buffer[i];
        auto& invMass1 = invMass1Buffer[i];
        auto& position0 = position0Buffer[i];
        auto& position1 = position1Buffer[i];
        auto& orientation0 = orientation0Buffer[i];
        auto& orientation1 = orientation1Buffer[i];

        glm::mat3 invInertiaTensor0(0);
        if (dynamic0 && !dynamic0->isKinematic) {
            invMass0 = dynamic0->invMass;
            invInertiaTensor0 = dynamic0->invInertiaTensorWorld;
        }

        glm::mat3 invInertiaTensor1(0);
        if (dynamic1 && !dynamic1->isKinematic) {
            invMass1 = dynamic1->invMass;
            invInertiaTensor1 = dynamic1->invInertiaTensorWorld;
        }

        angular0t = invInertiaTensor0 * angular0;
        angular1t = invInertiaTensor1 * angular1;

        invEffMass = glm::dot(angular0, angular0t) + glm::dot(angular1, angular1t);
        if (!(flags & Constraint1D::ANGULAR)) {
            invEffMass += glm::dot(linear, linear) * (invMass0 + invMass1);
        }

        position0 = transform0->position;
        position1 = transform1->position;
        orientation0 = transform0->orientation;
        orientation1 = transform1->orientation;
    }

    // correct position error
    const auto factor = _mm_set1_ps(0.1f);
    const auto half = _mm_set1_ps(0.5f);

    for (int i = 0; i < size; i += 4) {
        const auto invEffMass = &invEffMassBuffer[i];
        const auto invEffMassW = _mm_load_ps(invEffMass);
        const auto invEffMassMask = _mm_cmpneq_ps(invEffMassW, _mm_setzero_ps());
        if (isZero(invEffMassMask)) continue;

        const auto c = &cBuffer[i];
        const auto cW = _mm_load_ps(c);
        const auto cMask =  _mm_cmpneq_ps(cW, _mm_setzero_ps());
        if (isZero(cMask)) continue;

        auto flags = &flagsBuffer[i];
        const unsigned int flagsW = *reinterpret_cast<unsigned int*>(flags);
        if ((flagsW & SOFT_W) == SOFT_W) continue;

        auto orientation0 = &orientation0Buffer[i];
        auto orientation1 = &orientation1Buffer[i];

        auto orientation0W = QuatW(orientation0);
        auto orientation1W = QuatW(orientation1);

        const auto angular0t = &angular0tBuffer[i];
        const auto angular1t = &angular1tBuffer[i];

        auto angular0tW = Vec3W(angular0t);
        auto angular1tW = Vec3W(angular1t);

        auto lambdaW = factor * cW / invEffMassW;
        if (flagsW & LIMITED_W) {
            auto minW = _mm_load_ps(&minBuffer[i]);
            auto maxW = _mm_load_ps(&maxBuffer[i]);

            lambdaW = _mm_min_ps(_mm_max_ps(lambdaW, minW), maxW);
        }

        auto softW = flagsW & SOFT_W;
        unsigned char* softWBytes = reinterpret_cast<unsigned char*>(&softW);
        auto softMask = _mm_cmpeq_epi32(_mm_setzero_si128(), _mm_set_epi32(softWBytes[3], softWBytes[2], softWBytes[1], softWBytes[0]));
        auto mask = _mm_and_ps(cMask, invEffMassMask);
        mask = _mm_and_ps(mask, _mm_castsi128_ps(softMask));
        lambdaW = _mm_blendv_ps(_mm_setzero_ps(), lambdaW, mask);

        if ((flagsW & ANGULAR_W) != ANGULAR_W) {
            const auto linearW = Vec3W(&linearBuffer[i]);

            const auto invMass0W = _mm_load_ps(&invMass0Buffer[i]);
            const auto invMass1W = _mm_load_ps(&invMass1Buffer[i]);

            auto position0 = &position0Buffer[i];
            auto position1 = &position1Buffer[i];

            auto position0W = Vec3W(position0);
            auto position1W = Vec3W(position1);

            position0W += lambdaW * invMass0W * linearW;
            position1W -= lambdaW * invMass1W * linearW;

            position0W.store(position0);
            position1W.store(position1);
        }

        orientation0W += half * QuatW(_mm_setzero_ps(), lambdaW * angular0tW) * orientation0W;
        orientation0W = normalize(orientation0W);

        orientation1W -= half * QuatW(_mm_setzero_ps(), lambdaW * angular1tW) * orientation1W;
        orientation1W = normalize(orientation1W);

        orientation0W.store(orientation0);
        orientation1W.store(orientation1);
    }

    for (int i = 0; i < size; ++i) {
        auto [dynamic0, dynamic1] = dynamicComponentsBuffer[i];
        auto [transform0, transform1] = transformComponentsBuffer[i];
        const auto& flags = flagsBuffer[i];
        const auto& c = cBuffer[i];
        auto& totalLambda = totalLambdaBuffer[i];
        const auto& linear = linearBuffer[i];
        const auto& angular0t = angular0tBuffer[i];
        const auto& angular1t = angular1tBuffer[i];
        const auto& position0 = position0Buffer[i];
        const auto& position1 = position1Buffer[i];
        const auto& orientation0 = orientation0Buffer[i];
        const auto& orientation1 = orientation1Buffer[i];

        transform0->position = position0;
        transform0->orientation = orientation0;

        transform1->position = position1;
        transform1->orientation = orientation1;

        // warm start

        if (flags & Constraint1D::SOFT || glm::abs(c) > 1e-4 || glm::abs(totalLambda) > 10000) continue;

        totalLambda = totalLambda * 0.5f;
        if (dynamic0 && !dynamic0->isKinematic) {
            if (!(flags & Constraint1D::ANGULAR))
                dynamic0->velocity += totalLambda * dynamic0->invMass * linear;
            dynamic0->angularVelocity += totalLambda * angular0t;
        }

        if (dynamic1 && !dynamic1->isKinematic) {
            if (!(flags & Constraint1D::ANGULAR))
                dynamic1->velocity -= totalLambda * dynamic1->invMass * linear;
            dynamic1->angularVelocity -= totalLambda * angular1t;
        }
    }
}

void physecs::Constraint1DSoa::solve(bool useBias, float timeStep) {
    for (int i = 0; i < size; i++) {
        auto [dynamic0, dynamic1] = dynamicComponentsBuffer[i];
        auto& linear = linearBuffer[i];
        auto& angular0 = angular0Buffer[i];
        auto& angular1 = angular1Buffer[i];
        auto& angular0t = angular0tBuffer[i];
        auto& angular1t = angular1tBuffer[i];
        const auto& c = cBuffer[i];
        const auto& flags = flagsBuffer[i];
        const auto& min = minBuffer[i];
        const auto& max = maxBuffer[i];
        const auto& frequency = frequencyBuffer[i];
        const auto& dampingRatio = dampingRatioBuffer[i];
        const auto& targetVelocity = targetVelocityBuffer[i];
        auto& invEffMass = invEffMassBuffer[i];
        auto& totalLambda = totalLambdaBuffer[i];

        if (!invEffMass) continue;

        glm::vec3 velocity0(0), angularVelocity0(0);
        if (dynamic0 && !dynamic0->isKinematic) {
            velocity0 = dynamic0->velocity;
            angularVelocity0 = dynamic0->angularVelocity;
        }

        glm::vec3 velocity1(0), angularVelocity1(0);
        if (dynamic1 && !dynamic1->isKinematic) {
            velocity1 = dynamic1->velocity;
            angularVelocity1 = dynamic1->angularVelocity;
        }

        float relativeVelocity = glm::dot(-angular0, angularVelocity0) + glm::dot(angular1, angularVelocity1);
        if (!(flags & Constraint1D::ANGULAR)) {
            relativeVelocity += glm::dot(-linear, velocity0) + glm::dot(linear, velocity1);
        }

        float lambda;
        if (flags & Constraint1D::SOFT) {
            float angularFreq = 2.f * glm::pi<float>() * frequency;
            float stiffness = angularFreq * angularFreq / invEffMass;
            float damping = 2.f * angularFreq * dampingRatio / invEffMass;
            float gamma = 1.f / (damping + timeStep * stiffness);
            float beta = timeStep * stiffness / (damping + timeStep * stiffness);
            lambda = (relativeVelocity + beta * c / timeStep) / (invEffMass + gamma / timeStep);
        } else {
            lambda = (relativeVelocity - targetVelocity + (useBias ? 0.2f * c / timeStep : 0)) / invEffMass;
        }

        if (flags & Constraint1D::LIMITED) {
            float prevLambda = totalLambda;
            totalLambda += lambda;
            totalLambda = glm::clamp(totalLambda, min, max);
            lambda = totalLambda - prevLambda;
        }
        else {
            totalLambda += lambda;
        }

        if (dynamic0 && !dynamic0->isKinematic) {
            if (!(flags & Constraint1D::ANGULAR))
                dynamic0->velocity += lambda * dynamic0->invMass * linear;
            dynamic0->angularVelocity += lambda * angular0t;
        }

        if (dynamic1 && !dynamic1->isKinematic) {
            if (!(flags & Constraint1D::ANGULAR))
                dynamic1->velocity -= lambda * dynamic1->invMass * linear;
            dynamic1->angularVelocity -= lambda * angular1t;
        }
    }
}

void physecs::Constraint1DSoa::solveSimd(float timeStep) {
    for (int i = 0; i < size; ++i) {
        auto [dynamic0, dynamic1] = dynamicComponentsBuffer[i];
        auto& velocity0 = velocity0Buffer[i];
        auto& velocity1 = velocity1Buffer[i];
        auto& angularVelocity0 = angularVelocity0Buffer[i];
        auto& angularVelocity1 = angularVelocity1Buffer[i];

        if (dynamic0 && !dynamic0->isKinematic) {
            velocity0 = dynamic0->velocity;
            angularVelocity0 = dynamic0->angularVelocity;
        }

        if (dynamic1 && !dynamic1->isKinematic) {
            velocity1 = dynamic1->velocity;
            angularVelocity1 = dynamic1->angularVelocity;
        }
    }

    const auto biasFactor = _mm_set1_ps(0.2 / timeStep);
    const auto timeStepW = _mm_set1_ps(timeStep);
    const auto one = _mm_set1_ps(1.f);
    const auto two = _mm_set1_ps(2.f);
    const auto twoPi = _mm_set1_ps(2.f * glm::pi<float>());

    for (int i = 0; i < size; i += 4) {
        const auto invEffMass = &invEffMassBuffer[i];
        auto invEffMassW = _mm_load_ps(invEffMass);
        auto invEffMassMask = _mm_cmpneq_ps(invEffMassW, _mm_setzero_ps());
        if (isZero(invEffMassMask)) continue;

        auto angularVelocity0 = &angularVelocity0Buffer[i];
        auto angularVelocity1 = &angularVelocity1Buffer[i];

        auto angularVelocity0W = Vec3W(angularVelocity0);
        auto angularVelocity1W = Vec3W(angularVelocity1);

        const auto angular0 = &angular0Buffer[i];
        const auto angular1 = &angular1Buffer[i];

        auto angular0W = Vec3W(angular0);
        auto angular1W = Vec3W(angular1);

        const auto targetVelocity = &targetVelocityBuffer[i];
        auto targetVelocityW = _mm_load_ps(targetVelocity);

        const auto c = &cBuffer[i];
        auto cW = _mm_load_ps(c);

        auto flags = &flagsBuffer[i];
        const unsigned int flagsW = *reinterpret_cast<unsigned int*>(flags);

        glm::vec3 *velocity0, *velocity1;
        Vec3W velocity0W, velocity1W, linearW;

        auto relativeVelocityW = dotW(angular1W, angularVelocity1W) - dotW(angular0W, angularVelocity0W);
        if ((flagsW & ANGULAR_W) != ANGULAR_W) {
            velocity0 = &velocity0Buffer[i];
            velocity1 = &velocity1Buffer[i];

            velocity1W = Vec3W(velocity1);
            velocity0W = Vec3W(velocity0);

            linearW = Vec3W(&linearBuffer[i]);

            relativeVelocityW += dotW(linearW, velocity1W) - dotW(linearW, velocity0W);
        }

        const auto effMass = one / invEffMassW;
        auto lambdaW = (relativeVelocityW - targetVelocityW + biasFactor * cW) * effMass;
        if (flagsW & SOFT_W) {
            const auto frequency = &frequencyBuffer[i];
            const auto dampingRatio = &dampingRatioBuffer[i];
            auto frequencyW = _mm_load_ps(frequency);
            auto dampingRatioW = _mm_load_ps(dampingRatio);

            auto angularFreq = twoPi * frequencyW;
            auto stiffness = angularFreq * angularFreq * effMass;
            auto damping = two * angularFreq * dampingRatioW * effMass;
            auto gamma = one / (damping + timeStepW * stiffness);
            auto beta = timeStepW * stiffness * gamma;
            auto lambdaSoft = (relativeVelocityW + beta * cW / timeStepW) / (invEffMassW + gamma / timeStepW);

            auto softW = flagsW & SOFT_W;
            unsigned char* softWBytes = reinterpret_cast<unsigned char*>(&softW);
            auto softMask = _mm_cmpeq_epi32(_mm_setzero_si128(), _mm_set_epi32(softWBytes[3], softWBytes[2], softWBytes[1], softWBytes[0]));
            lambdaW = _mm_blendv_ps(lambdaSoft, lambdaW, _mm_castsi128_ps(softMask));
        }

        auto totalLambda = &totalLambdaBuffer[i];
        auto totalLambdaW = _mm_load_ps(totalLambda);
        auto prevLambda = totalLambdaW;

        if (flagsW & LIMITED_W) {
            auto minW = _mm_load_ps(&minBuffer[i]);
            auto maxW = _mm_load_ps(&maxBuffer[i]);

            totalLambdaW += lambdaW;
            totalLambdaW = _mm_min_ps(_mm_max_ps(totalLambdaW, minW), maxW);
            lambdaW = totalLambdaW - prevLambda;
        }
        else {
            totalLambdaW += lambdaW;
        }

        totalLambdaW = _mm_blendv_ps(prevLambda, totalLambdaW, invEffMassMask);

        _mm_store_ps(totalLambda, totalLambdaW);

        if ((flagsW & ANGULAR_W) != ANGULAR_W) {
            auto invMass0W = _mm_load_ps(&invMass0Buffer[i]);
            auto invMass1W = _mm_load_ps(&invMass1Buffer[i]);

            velocity0W += lambdaW * invMass0W * linearW;
            velocity1W -= lambdaW * invMass1W * linearW;

            velocity0W.store(velocity0);
            velocity1W.store(velocity1);
        }

        const auto angular0t = &angular0tBuffer[i];
        const auto angular1t = &angular1tBuffer[i];
        auto angular0tW = Vec3W(angular0t);
        auto angular1tW = Vec3W(angular1t);

        angularVelocity0W += lambdaW * angular0tW;
        angularVelocity1W -= lambdaW * angular1tW;

        angularVelocity0W.store(angularVelocity0);
        angularVelocity1W.store(angularVelocity1);
    }

    for (int i = 0; i < size; ++i) {
        const auto& invEffMass = invEffMassBuffer[i];

        if (!invEffMass) continue;

        auto [dynamic0, dynamic1] = dynamicComponentsBuffer[i];
        const auto& flags = flagsBuffer[i];
        const auto& velocity0 = velocity0Buffer[i];
        const auto& velocity1 = velocity1Buffer[i];
        const auto& angularVelocity0 = angularVelocity0Buffer[i];
        const auto& angularVelocity1 = angularVelocity1Buffer[i];

        if (dynamic0 && !dynamic0->isKinematic) {
            if (!(flags & Constraint1D::ANGULAR))
                dynamic0->velocity = velocity0;
            dynamic0->angularVelocity = angularVelocity0;
        }

        if (dynamic1 && !dynamic1->isKinematic) {
            if (!(flags & Constraint1D::ANGULAR))
                dynamic1->velocity = velocity1;
            dynamic1->angularVelocity = angularVelocity1;
        }
    }
}

void physecs::Constraint1DContainer::preSolve() {
    for (auto& constraintColor : constraintColors) {
        constraintColor.preSolveSimd();
    }
    sequential.preSolve();
}

void physecs::Constraint1DContainer::solve(bool useBias, float timeStep) {
    for (auto& constraintColor : constraintColors) {
        constraintColor.solveSimd(timeStep);
    }
    sequential.solve(useBias, timeStep);
}

void physecs::Constraint1DContainer::pushBack(int count, entt::entity entity0, entt::entity entity1, TransformComponent &transform0, TransformComponent &transform1, RigidBodyDynamicComponent *dynamic0, RigidBodyDynamicComponent *dynamic1) {
    auto& colors0 = colorBitsets[entity0];
    auto& colors1 = colorBitsets[entity1];
    for (int j = 0; j < count; ++j) {
        const auto colorsUnion = colors0 | colors1;
        unsigned long i;
        if (_BitScanForward(&i, ~colorsUnion)) {
            colors0 |= 1 << i;
            colors1 |= 1 << i;

            auto& constraintColor = constraintColors[i];
            mappers.emplace_back(&constraintColor, constraintColor.getSize());
            constraintColor.pushBack(transform0, transform1, dynamic0, dynamic1);
        }
        else {
            mappers.emplace_back(&sequential, sequential.getSize());
            sequential.pushBack(transform0, transform1, dynamic0, dynamic1);
        }
    }
}

void physecs::Constraint1DContainer::fillDefaults() {
    for (auto& constraintColor : constraintColors) {
        constraintColor.fillDefaults();
    }
    sequential.fillDefaults();
}

void physecs::Constraint1DContainer::clear() {
    for (auto& constraintColor : constraintColors) {
        constraintColor.clear();
    }
    sequential.clear();
    mappers.clear();
    colorBitsets.clear();
}
