#pragma once
#include "Joint.h"

namespace physecs {
    struct FixedJoint;

    struct FixedJointDef {
        struct Data {};

        struct Cache {
            float pointLambda[3] = {};
            float angularLambda[3] = {};
        };

        using Layout = ConstraintLayout<
            ConstraintBlock<NONE, 3, &Cache::pointLambda>,
            ConstraintBlock<ANGULAR, 3, &Cache::angularLambda>>;

        using Base = JointImpl<FixedJoint, Layout, Cache, Data>;
    };

    PHYSECS_DECLARE_JOINT_IMPL(FixedJoint);

    struct PHYSECS_API FixedJoint final : FixedJointDef::Base {
        static void makeConstraints(const JointWorldSpaceData& worldSpaceData, void* additionalData, Constraint1DWriter& constraints);
        using FixedJointDef::Base::Base;
    };
}

