#pragma once
#include "Joint.h"

namespace physecs {

    class PHYSECS_API ServoJoint final : public Joint {
        struct ServoJointData {
            float targetAngle = 0;
            float driveStiffness = 30;
            float driveDamping = 1;
        };

        struct ImpulseCache {
            float pointLambda[3] = {};
            float angularLambda[2] = {};
        };

        ServoJointData data;
        ImpulseCache impulseCache;

        static void makeConstraints(const JointWorldSpaceData& worldSpaceData, void* additionalData, Constraint1DWriter& constraints);
    public:
        void setTargetAngle(float angle);
        void setDriveStiffness(float stiffness);
        void setDriveDamping(float damping);
        ServoJoint(entt::entity entity0, glm::vec3 anchor0Pos, glm::quat anchor0Or, entt::entity entity1, glm::vec3 anchor1Pos, glm::quat anchor1Or) : Joint(entity0, anchor0Pos, anchor0Or, entity1, anchor1Pos, anchor1Or) {}
        JointSolverDesc getSolverDesc(entt::registry &registry, Constraint1DLayout& constraintLayout) override;
        void storeAccumulatedImpulses(Constraint1DReader& constraints) override;
    };

}
