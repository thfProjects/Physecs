#pragma once
#include "Joint.h"

namespace physecs {
    struct GearJoint;

    struct GearJointDef {
        struct Data {
            float gearRatio = 1;
            float persistentAngle0 = 0;
            float persistentAngle1 = 0;
            float slip = 0;
            bool isInitialized = false;
        };

        struct Cache {
            float lambda = 0;
        };

        using Layout = ConstraintLayout<
            ConstraintBlock<NONE, 1, &Cache::lambda>>;

        using Base = JointImpl<GearJoint, Layout, Cache, Data>;
    };

    PHYSECS_DECLARE_JOINT_IMPL(GearJoint);

    struct PHYSECS_API GearJoint final : GearJointDef::Base {
        static void makeConstraints(const JointWorldSpaceData& worldSpaceData, void* additionalData, Constraint1DWriter& constraints);
        using GearJointDef::Base::Base;

        void setGearRatio(float gearRatio);
    };
}
