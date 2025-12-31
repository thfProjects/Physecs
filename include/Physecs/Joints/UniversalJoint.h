#pragma once
#include "Joint.h"

namespace physecs {

    class PHYSECS_API UniversalJoint final : public Joint {
        constexpr  static int numConstraints = 2;
        static void makeConstraints(JointWorldSpaceData& worldSpaceData, void* additionalData, Constraint1DWriter& constraints);
    public:
        UniversalJoint(entt::entity entity0, glm::vec3 anchor0Pos, glm::quat anchor0Or, entt::entity entity1, glm::vec3 anchor1Pos, glm::quat anchor1Or) : Joint(entity0, anchor0Pos, anchor0Or, entity1, anchor1Pos, anchor1Or) {}
        JointSolverDesc getSolverDesc(entt::registry &registry, Constraint1DLayout& constraintLayout) override;
    };
}
