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

        capacity = newCapacity;
    }

    transformComponentsBuffer[size] = { &transform0, &transform1 };
    dynamicComponentsBuffer[size] = { dynamic0, dynamic1 };
    linearBuffer[size] = glm::vec3(0);
    angular0Buffer[size] = glm::vec3(0);
    angular1Buffer[size] = glm::vec3(0);
    targetVelocityBuffer[size] = 0;
    cBuffer[size] = 0;
    minBuffer[size] = std::numeric_limits<float>::lowest();
    maxBuffer[size] = std::numeric_limits<float>::max();
    flagsBuffer[size] = 0;
    frequencyBuffer[size] = 0;
    dampingRatioBuffer[size] = 0;
    angular0tBuffer[size] = glm::vec3(0);
    angular1tBuffer[size] = glm::vec3(0);
    invEffMassBuffer[size] = 0;
    totalLambdaBuffer[size] = 0;

    ++size;
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

        float invMass0 = 0;
        glm::mat3 invInertiaTensor0(0);
        if (dynamic0 && !dynamic0->isKinematic) {
            invMass0 = dynamic0->invMass;
            invInertiaTensor0 = dynamic0->invInertiaTensorWorld;
        }

        float invMass1 = 0;
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
        if (flags & Constraint1D::SOFT || !c || !invEffMass) continue;

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
    for (int i = 0; i < size; i += 4) {
        auto dynamics = &dynamicComponentsBuffer[i];
        const auto linear = &linearBuffer[i];
        const auto angular0 = &angular0Buffer[i];
        const auto angular1 = &angular1Buffer[i];
        const auto angular0t = &angular0tBuffer[i];
        const auto angular1t = &angular1tBuffer[i];
        const auto c = &cBuffer[i];
        auto flags = &flagsBuffer[i];
        const auto min = &minBuffer[i];
        const auto max = &maxBuffer[i];
        const auto frequency = &frequencyBuffer[i];
        const auto dampingRatio = &dampingRatioBuffer[i];
        const auto targetVelocity = &targetVelocityBuffer[i];
        const auto invEffMass = &invEffMassBuffer[i];
        auto totalLambda = &totalLambdaBuffer[i];

        glm::vec3 velocity0[4] = { glm::vec3(0), glm::vec3(0), glm::vec3(0), glm::vec3(0) };
        glm::vec3 angularVelocity0[4] = { glm::vec3(0), glm::vec3(0), glm::vec3(0), glm::vec3(0) };
        glm::vec3 velocity1[4] = { glm::vec3(0), glm::vec3(0), glm::vec3(0), glm::vec3(0) };
        glm::vec3 angularVelocity1[4] = { glm::vec3(0), glm::vec3(0), glm::vec3(0), glm::vec3(0) };

        for (int j = 0; j < 4 && i + j < size; ++j) {
            auto [dynamic0, dynamic1] = dynamics[j];

            if (dynamic0 && !dynamic0->isKinematic) {
                velocity0[j] = dynamic0->velocity;
                angularVelocity0[j] = dynamic0->angularVelocity;
            }

            if (dynamic1 && !dynamic1->isKinematic) {
                velocity1[j] = dynamic1->velocity;
                angularVelocity1[j] = dynamic1->angularVelocity;
            }
        }

        auto velocity0W = vec3W(velocity0);
        auto angularVelocity0W = vec3W(angularVelocity0);
        auto velocity1W = vec3W(velocity1);
        auto angularVelocity1W = vec3W(angularVelocity1);

        auto angular0W = vec3W(angular0);
        auto angular1W = vec3W(angular1);
        auto linearW = vec3W(linear);

        auto targetVelocityW = _mm_load_ps(targetVelocity);
        auto cW = _mm_load_ps(c);
        auto invEffMassW = _mm_load_ps(invEffMass);

        const unsigned int flagsW = static_cast<unsigned char>(flags[0]) | static_cast<unsigned char>(flags[1]) << 8 | static_cast<unsigned char>(flags[2]) << 16 | static_cast<unsigned char>(flags[3]) << 24;

        auto relativeVelocityW = dotW(angular1W, angularVelocity1W) - dotW(angular0W, angularVelocity0W);
        if ((flagsW & ANGULAR_W) != ANGULAR_W) {
            relativeVelocityW += dotW(linearW, velocity1W) - dotW(linearW, velocity0W);
        }

        auto lambdaW = (relativeVelocityW - targetVelocityW + _mm_set1_ps(0.2 / timeStep) * cW) / invEffMassW;
        if (flagsW & SOFT_W) {
            auto frequencyW = _mm_load_ps(frequency);
            auto dampingRatioW = _mm_load_ps(dampingRatio);
            auto timeStepW = _mm_set1_ps(timeStep);

            auto angularFreq = _mm_set1_ps(2.f * glm::pi<float>()) * frequencyW;
            auto stiffness = angularFreq * angularFreq / invEffMassW;
            auto damping = _mm_set1_ps(2.f) * angularFreq * dampingRatioW / invEffMassW;
            auto gamma = _mm_set1_ps(1.f) / (damping + timeStepW * stiffness);
            auto beta = timeStepW * stiffness / (damping + timeStepW * stiffness);
            auto lambdaSoft = (relativeVelocityW + beta * cW / timeStepW) / (invEffMassW + gamma / timeStepW);

            auto mask = _mm_set_epi32(flags[3] & Constraint1D::SOFT ? 0xffffffff : 0, flags[2] & Constraint1D::SOFT ? 0xffffffff : 0, flags[1] & Constraint1D::SOFT ? 0xffffffff : 0, flags[0] & Constraint1D::SOFT ? 0xffffffff : 0);
            lambdaW = _mm_blendv_ps(lambdaW, lambdaSoft, _mm_castsi128_ps(mask));
        }

        auto totalLambdaW = _mm_load_ps(totalLambda);

        if (flagsW & LIMITED_W) {
            auto minW = _mm_load_ps(min);
            auto maxW = _mm_load_ps(max);

            auto prevLambda = totalLambdaW;
            totalLambdaW += lambdaW;
            totalLambdaW = _mm_min_ps(_mm_max_ps(totalLambdaW, minW), maxW);
            lambdaW = totalLambdaW - prevLambda;
        }
        else {
            totalLambdaW += lambdaW;
        }

        for (int j = 0; j < 4 && i + j < size; ++j) {
            if (!invEffMass[j]) continue;

            auto [dynamic0, dynamic1] = dynamics[j];

            if (dynamic0 && !dynamic0->isKinematic) {
                if (!(flags[j] & Constraint1D::ANGULAR))
                    dynamic0->velocity += lambdaW.m128_f32[j] * dynamic0->invMass * linear[j];
                dynamic0->angularVelocity += lambdaW.m128_f32[j] * angular0t[j];
            }

            if (dynamic1 && !dynamic1->isKinematic) {
                if (!(flags[j] & Constraint1D::ANGULAR))
                    dynamic1->velocity -= lambdaW.m128_f32[j] * dynamic1->invMass * linear[j];
                dynamic1->angularVelocity -= lambdaW.m128_f32[j] * angular1t[j];
            }

            totalLambda[j] = totalLambdaW.m128_f32[j];
        }
    }
}

void physecs::Constraint1DContainer::preSolve() {
    for (auto& constraintColor : constraintColors) {
        constraintColor.preSolve();
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
    if (!colorBitsets.contains(entity0)) colorBitsets.emplace(entity0, 0);
    if (!colorBitsets.contains(entity1)) colorBitsets.emplace(entity1, 0);
    auto& colors0 = colorBitsets[entity0];
    auto& colors1 = colorBitsets[entity1];
    for (int j = 0; j < count; ++j) {
        const auto colorsUnion = colors0 | colors1;
        unsigned long i;
        if (_BitScanForward(&i, ~colorsUnion)) {
            colors0 |= 1 << i;
            colors1 |= 1 << i;

            if (i == constraintColors.size()) {
                constraintColors.push_back({});
            }

            auto& constraintColor = constraintColors[i];
            mappers.emplace_back(static_cast<int>(i), constraintColor.getSize());
            constraintColor.pushBack(transform0, transform1, dynamic0, dynamic1);
        }
        else {
            mappers.push_back({ -1, sequential.getSize() });
            sequential.pushBack(transform0, transform1, dynamic0, dynamic1);
        }
    }
}

void physecs::Constraint1DContainer::clear() {
    for (auto& constraintColor : constraintColors) {
        constraintColor.clear();
    }
    sequential.clear();
    mappers.clear();
    colorBitsets.clear();
}
