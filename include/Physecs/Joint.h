#pragma once

#include "PhysecsAPI.h"
#include <entt.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

namespace physecs {
    struct Constraint1D;

    class PHYSECS_API Joint {
    protected:
        entt::entity entity0;
        entt::entity entity1;
        glm::vec3 anchor0Pos;
        glm::quat anchor0Or;
        glm::vec3 anchor1Pos;
        glm::quat anchor1Or;
        int numConstraints;

        Joint(entt::entity entity0, glm::vec3 anchor0Pos, glm::quat anchor0Or, entt::entity entity1, glm::vec3 anchor1Pos, glm::quat anchor1Or, int numConstraints = 0) : entity0(entity0), entity1(entity1), anchor0Pos(anchor0Pos), anchor0Or(anchor0Or), anchor1Pos(anchor1Pos), anchor1Or(anchor1Or), numConstraints(numConstraints) {}
        void getJointData(entt::registry& registry, glm::vec3& p0, glm::vec3& p1, glm::vec3& r0, glm::vec3& r1, glm::mat3& u0, glm::mat3& u1);
    public:
        virtual void makeConstraints(Constraint1D* constraints, entt::registry& registry) = 0;
        virtual void calculateNumConstraints(entt::registry& registry) {}
        entt::entity getEntity0() { return entity0; }
        entt::entity getEntity1() { return entity1; }
        int getNumConstraints() { return numConstraints; }

        virtual ~Joint() = default;
    };
}
