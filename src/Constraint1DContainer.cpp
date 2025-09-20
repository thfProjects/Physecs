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

        velocity0Buffer.reallocate(capacity, newCapacity);
        velocity1Buffer.reallocate(capacity, newCapacity);
        angularVelocity0Buffer.reallocate(capacity, newCapacity);
        angularVelocity1Buffer.reallocate(capacity, newCapacity);
        invMass0Buffer.reallocate(capacity, newCapacity);
        invMass1Buffer.reallocate(capacity, newCapacity);

        capacity = newCapacity;
    }

    transformComponentsBuffer[size] = { &transform0, &transform1 };
    dynamicComponentsBuffer[size] = { dynamic0, dynamic1 };

    ++size;
}

void physecs::Constraint1DSoa::fillDefaults() {
    std::fill_n(linearBuffer.get(), size, glm::vec3(0));
    std::fill_n(angular0Buffer.get(), size, glm::vec3(0));
    std::fill_n(angular1Buffer.get(), size, glm::vec3(0));
    std::fill_n(targetVelocityBuffer.get(), size, 0);
    std::fill_n(cBuffer.get(), size, 0);
    std::fill_n(minBuffer.get(), size, std::numeric_limits<float>::lowest());
    std::fill_n(maxBuffer.get(), size, std::numeric_limits<float>::max());
    std::fill_n(flagsBuffer.get(), size, 0);
    std::fill_n(frequencyBuffer.get(), size, 0);
    std::fill_n(dampingRatioBuffer.get(), size, 0);
    std::fill_n(angular0tBuffer.get(), size, glm::vec3(0));
    std::fill_n(angular1tBuffer.get(), size, glm::vec3(0));
    std::fill_n(invEffMassBuffer.get(), size, 0);
    std::fill_n(totalLambdaBuffer.get(), size, 0);

    std::fill_n(velocity0Buffer.get(), size, glm::vec3(0));
    std::fill_n(velocity1Buffer.get(), size, glm::vec3(0));
    std::fill_n(angularVelocity0Buffer.get(), size, glm::vec3(0));
    std::fill_n(angularVelocity1Buffer.get(), size, glm::vec3(0));
    std::fill_n(invMass0Buffer.get(), size, 0);
    std::fill_n(invMass0Buffer.get(), size, 0);
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

    for (int i = 0; i < size; i += 4) {
        const auto invEffMass = &invEffMassBuffer[i];

        auto invEffMassW = _mm_load_ps(invEffMass);

        auto invEffMassMask = _mm_cmpneq_ps(invEffMassW, _mm_setzero_ps());

        if (_mm_testz_si128(_mm_castps_si128(invEffMassMask), _mm_castps_si128(invEffMassMask))) {
            continue;
        }

        const auto linear = &linearBuffer[i];
        const auto angular0 = &angular0Buffer[i];
        const auto angular1 = &angular1Buffer[i];
        const auto c = &cBuffer[i];
        auto flags = &flagsBuffer[i];
        const auto targetVelocity = &targetVelocityBuffer[i];

        auto totalLambda = &totalLambdaBuffer[i];

        auto velocity0 = &velocity0Buffer[i];
        auto velocity1 = &velocity1Buffer[i];
        auto angularVelocity0 = &angularVelocity0Buffer[i];
        auto angularVelocity1 = &angularVelocity1Buffer[i];

        auto velocity0W = vec3W(velocity0);
        auto angularVelocity0W = vec3W(angularVelocity0);
        auto velocity1W = vec3W(velocity1);
        auto angularVelocity1W = vec3W(angularVelocity1);

        auto angular0W = vec3W(angular0);
        auto angular1W = vec3W(angular1);
        auto linearW = vec3W(linear);

        auto targetVelocityW = _mm_load_ps(targetVelocity);
        auto cW = _mm_load_ps(c);

        const unsigned int flagsW = flags[0] | flags[1] << 8 | flags[2] << 16 | flags[3] << 24;

        auto relativeVelocityW = dotW(angular1W, angularVelocity1W) - dotW(angular0W, angularVelocity0W);
        if ((flagsW & ANGULAR_W) != ANGULAR_W) {
            relativeVelocityW += dotW(linearW, velocity1W) - dotW(linearW, velocity0W);
        }

        auto lambdaW = (relativeVelocityW - targetVelocityW + _mm_set1_ps(0.2 / timeStep) * cW) / invEffMassW;
        if (flagsW & SOFT_W) {
            const auto frequency = &frequencyBuffer[i];
            const auto dampingRatio = &dampingRatioBuffer[i];
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

        auto invMass0W = _mm_load_ps(&invMass0Buffer[i]);
        auto invMass1W = _mm_load_ps(&invMass1Buffer[i]);
        const auto angular0t = &angular0tBuffer[i];
        const auto angular1t = &angular1tBuffer[i];
        auto angular0tW = vec3W(angular0t);
        auto angular1tW = vec3W(angular1t);

        if ((flagsW & ANGULAR_W) != ANGULAR_W) {
            velocity0W += lambdaW * invMass0W * linearW;
            velocity1W -= lambdaW * invMass1W * linearW;
        }

        angularVelocity0W += lambdaW * angular0tW;
        angularVelocity1W -= lambdaW * angular1tW;

        velocity0W.store(velocity0);
        velocity1W.store(velocity1);
        angularVelocity0W.store(angularVelocity0);
        angularVelocity1W.store(angularVelocity1);

        _mm_store_ps(totalLambda, totalLambdaW);
    }

    for (int i = 0; i < size; ++i) {
        auto [dynamic0, dynamic1] = dynamicComponentsBuffer[i];
        const auto& flags = flagsBuffer[i];
        const auto& velocity0 = velocity0Buffer[i];
        const auto& velocity1 = velocity1Buffer[i];
        const auto& angularVelocity0 = angularVelocity0Buffer[i];
        const auto& angularVelocity1 = angularVelocity1Buffer[i];
        const auto& invEffMass = invEffMassBuffer[i];

        if (!invEffMass) continue;

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
