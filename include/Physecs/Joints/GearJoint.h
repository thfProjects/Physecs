#pragma once
#include "Joint.h"

namespace physecs {
    class PHYSECS_API GearJoint final : public Joint {

        struct GearJointData {
            float gearRatio = 1;
            float persistentAngle0 = 0;
            float persistentAngle1 = 0;
            float virtualAngle0 = 0;
            float virtualAngle1 = 0;
            bool isInitialized = false;
        };

        GearJointData data;

        constexpr  static int numConstraints = 1;
        static void makeConstraints(JointWorldSpaceData& worldSpaceData, void* additionalData, Constraint1DViewer constraints);
    public:
        void setGearRatio(float gearRatio);
        GearJoint(entt::entity entity0, glm::vec3 anchor0Pos, glm::quat anchor0Or, entt::entity entity1, glm::vec3 anchor1Pos, glm::quat anchor1Or) : Joint(entity0, anchor0Pos, anchor0Or, entity1, anchor1Pos, anchor1Or) {}
        JointSolverData getSolverData(entt::registry &registry) override;
    };
}
