#include "Constraint1DContainer.h"
#include "Transform.h"
#include <Constraint1D.h>

void physecs::Constraint1DContainer::pushBack(TransformComponent &transform0, TransformComponent &transform1, RigidBodyDynamicComponent *dynamic0, RigidBodyDynamicComponent *dynamic1) {
    transformComponentsBuffer.emplace_back(transform0, transform1);
    dynamicComponentsBuffer.emplace_back(dynamic0, dynamic1);
    linearBuffer.emplace_back(0);
    angular0Buffer.emplace_back(0);
    angular1Buffer.emplace_back(0);
    targetVelocityBuffer.emplace_back(0);
    cBuffer.emplace_back(0);
    minBuffer.emplace_back(std::numeric_limits<float>::lowest());
    maxBuffer.emplace_back(std::numeric_limits<float>::max());
    flagsBuffer.emplace_back(0);
    frequencyBuffer.emplace_back(0);
    dampingRatioBuffer.emplace_back(0);
    angular0tBuffer.emplace_back(0);
    angular1tBuffer.emplace_back(0);
    invEffMassBuffer.emplace_back(0);
    totalLambdaBuffer.emplace_back(0);
}

void physecs::Constraint1DContainer::clear() {
    transformComponentsBuffer.clear();
    dynamicComponentsBuffer.clear();
    linearBuffer.clear();
    angular0Buffer.clear();
    angular1Buffer.clear();
    targetVelocityBuffer.clear();
    cBuffer.clear();
    minBuffer.clear();
    maxBuffer.clear();
    flagsBuffer.clear();
    frequencyBuffer.clear();
    dampingRatioBuffer.clear();
    angular0tBuffer.clear();
    angular1tBuffer.clear();
    invEffMassBuffer.clear();
    totalLambdaBuffer.clear();
}

void physecs::Constraint1DContainer::preSolve() {
    for (int i = 0; i < transformComponentsBuffer.size(); i++) {
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
            transform0.position += lambda * invMass0 * linear;
            transform1.position -= lambda * invMass1 * linear;
        }

        transform0.orientation += 0.5f * glm::quat(0, lambda * angular0t) * transform0.orientation;
        transform0.orientation = glm::normalize(transform0.orientation);

        transform1.orientation -= 0.5f * glm::quat(0, lambda * angular1t) * transform1.orientation;
        transform1.orientation = glm::normalize(transform1.orientation);

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

void physecs::Constraint1DContainer::solve(bool useBias, float timeStep) {
    for (int i = 0; i < transformComponentsBuffer.size(); i++) {
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
