#pragma once

#include "PhysecsAPI.h"
#include <entt.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include "Components.h"

struct TransformComponent;

namespace physecs {
    struct Constraint1D;

    class PHYSECS_API Joint {
    protected:
        entt::entity entity0;
        entt::entity entity1;
        TransformComponent* transform0 = nullptr;
        TransformComponent* transform1 = nullptr;
        RigidBodyDynamicComponent* dynamic0 = nullptr;
        RigidBodyDynamicComponent* dynamic1 = nullptr;
        glm::vec3 anchor0Pos;
        glm::quat anchor0Or;
        glm::vec3 anchor1Pos;
        glm::quat anchor1Or;
        int numConstraints;

        Joint(entt::entity entity0, glm::vec3 anchor0Pos, glm::quat anchor0Or, entt::entity entity1, glm::vec3 anchor1Pos, glm::quat anchor1Or, int numConstraints = 0) : entity0(entity0), entity1(entity1), anchor0Pos(anchor0Pos), anchor0Or(anchor0Or), anchor1Pos(anchor1Pos), anchor1Or(anchor1Or), numConstraints(numConstraints) {}
        void getJointData(glm::vec3& p0, glm::vec3& p1, glm::vec3& r0, glm::vec3& r1, glm::mat3& u0, glm::mat3& u1) const;
    public:
        virtual void makeConstraints(Constraint1D* constraints, entt::registry& registry) = 0;
        virtual void calculateNumConstraints(entt::registry& registry) {}
        void update(entt::registry& registry);
        entt::entity getEntity0() const { return entity0; }
        entt::entity getEntity1() const { return entity1; }
        int getNumConstraints() const { return numConstraints; }

        virtual ~Joint() = default;
    };
}
