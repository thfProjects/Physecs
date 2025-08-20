#pragma once

#include "PhysecsAPI.h"
#include <entt.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include "Components.h"

struct TransformComponent;

namespace physecs {
    struct Constraint1D;

    struct JointWorldSpaceData {
        glm::vec3 p0;
        glm::vec3 p1;
        glm::vec3 r0;
        glm::vec3 r1;
        glm::mat3 u0;
        glm::mat3 u1;
    };

    typedef void (*MakeConstraintsFunc)(JointWorldSpaceData& worldSpaceData, void* additionalData, Constraint1D* constraints);

    struct JointSolverData {
        TransformComponent& transform0;
        TransformComponent& transform1;
        RigidBodyDynamicComponent* dynamic0;
        RigidBodyDynamicComponent* dynamic1;
        glm::vec3 anchor0Pos;
        glm::quat anchor0Or;
        glm::vec3 anchor1Pos;
        glm::quat anchor1Or;
        int numConstraints;
        void* additionalData;
        MakeConstraintsFunc makeConstraintsFunc;

        void calculateWorldSpaceData(JointWorldSpaceData& data) const;
    };

    class PHYSECS_API Joint {
    protected:
        entt::entity entity0;
        entt::entity entity1;
        glm::vec3 anchor0Pos;
        glm::quat anchor0Or;
        glm::vec3 anchor1Pos;
        glm::quat anchor1Or;

        Joint(entt::entity entity0, glm::vec3 anchor0Pos, glm::quat anchor0Or, entt::entity entity1, glm::vec3 anchor1Pos, glm::quat anchor1Or) : entity0(entity0), entity1(entity1), anchor0Pos(anchor0Pos), anchor0Or(anchor0Or), anchor1Pos(anchor1Pos), anchor1Or(anchor1Or) {}
    public:
        virtual JointSolverData getSolverData(entt::registry& registry) = 0;
        entt::entity getEntity0() const { return entity0; }
        entt::entity getEntity1() const { return entity1; }

        virtual ~Joint() = default;
    };
}
