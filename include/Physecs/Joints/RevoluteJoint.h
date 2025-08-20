#pragma once
#include "Joint.h"

namespace physecs {

    class PHYSECS_API RevoluteJoint final : public Joint {

        struct RevoluteJointData {
            bool driveEnabled = false;
            float driveVelocity = 0;
        };

        RevoluteJointData data;

        static void makeConstraints(JointWorldSpaceData& worldSpaceData, void* additionalData, Constraint1D* constraints);
    public:
        void setDriveEnabled(bool enabled);
        void setDriveVelocity(float velocity);
        RevoluteJoint(entt::entity entity0, glm::vec3 anchor0Pos, glm::quat anchor0Or, entt::entity entity1, glm::vec3 anchor1Pos, glm::quat anchor1Or) : Joint(entity0, anchor0Pos, anchor0Or, entity1, anchor1Pos, anchor1Or) {}
        JointSolverData getSolverData(entt::registry &registry) override;
    };

}
