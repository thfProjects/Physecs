#pragma once
#include "Joint.h"

namespace physecs {
    class PHYSECS_API FixedJoint final : public Joint {
    public:
        FixedJoint(entt::entity entity0, glm::vec3 anchor0Pos, glm::quat anchor0Or, entt::entity entity1, glm::vec3 anchor1Pos, glm::quat anchor1Or) : Joint(entity0, anchor0Pos, anchor0Or, entity1, anchor1Pos, anchor1Or, 4) {}
        void makeConstraints(Constraint1D* constraints, entt::registry& registry) override;
    };
}

