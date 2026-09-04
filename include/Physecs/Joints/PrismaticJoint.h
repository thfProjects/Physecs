#pragma once
#include "Joint.h"

namespace physecs {
    struct PrismaticJoint;

    struct PrismaticJointDef {
        struct Data {
            float upperLimit = 1;
            float lowerLimit = 0;
            bool makeUpperLimit = false;
            bool makeLowerLimit = false;
            bool driveEnabled = false;
            float targetPosition = 0;
            float driveStiffness = 5;
            float driveDamping = 1;
        };

        struct Cache {
            float translationLambda[2] = {};
            float angularLambda[3] = {};
            float upperLimitLambda = 0;
            float lowerLimitLambda = 0;
        };

        // angular blocks come first so orthogonalization never gives them a linear part
        using Layout = ConstraintLayout<
            ConstraintBlock<ANGULAR, 3, &Cache::angularLambda>,
            ConstraintBlock<NONE, 2, &Cache::translationLambda>,
            ConstraintBlock<LIMITED, 1, &Cache::upperLimitLambda, &Data::makeUpperLimit>,
            ConstraintBlock<LIMITED, 1, &Cache::lowerLimitLambda, &Data::makeLowerLimit>,
            ConstraintBlock<SOFT, 1, nullptr, &Data::driveEnabled>>;

        using Base = JointImpl<PrismaticJoint, Layout, Cache, Data>;
    };

    PHYSECS_DECLARE_JOINT_IMPL(PrismaticJoint);

    struct PHYSECS_API PrismaticJoint final : PrismaticJointDef::Base {
        static void makeConstraints(const JointWorldSpaceData& worldSpaceData, void* additionalData, Constraint1DDescriptor* constraintRows);

        void prepare(const entt::registry& registry);

        using PrismaticJointDef::Base::Base;

        void setUpperLimit(float upperLimit);
        void setLowerLimit(float lowerLimit);
        void setDriveEnabled(bool driveEnabled);
        void setTargetPosition(float targetPosition);
        void setDriveStiffness(float driveStiffness);
        void setDriveDamping(float driveDamping);
    };
}
