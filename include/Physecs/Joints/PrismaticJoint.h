#pragma once
#include "Joint.h"

namespace physecs {

    class PHYSECS_API PrismaticJoint final: public Joint {
        float upperLimit = 1;
        float lowerLimit = 0;
        bool makeUpperLimit = false;
        bool makeLowerLimit = false;
        bool driveEnabled = false;
        float targetPosition = 0;
        float driveStiffness = 5;
        float driveDamping = 1;
    public:
        void setUpperLimit(float upperLimit);
        void setLowerLimit(float lowerLimit);
        void setDriveEnabled(bool driveEnabled);
        void setTargetPosition(float targetPosition);
        void setDriveStiffness(float driveStiffness);
        void setDriveDamping(float driveDamping);
        PrismaticJoint(entt::entity entity0, glm::vec3 anchor0Pos, glm::quat anchor0Or, entt::entity entity1, glm::vec3 anchor1Pos, glm::quat anchor1Or) : Joint(entity0, anchor0Pos, anchor0Or, entity1, anchor1Pos, anchor1Or) {}
        void makeConstraints(Constraint1D* constraints) override;
        void calculateNumConstraints(entt::registry& registry) override;
    };
}
