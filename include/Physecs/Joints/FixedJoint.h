#pragma once
#include "Joint.h"

namespace physecs {
    struct FixedJoint;

    struct FixedJointDef {
        struct Cache {
            float pointLambda[3] = {};
            float angularLambda[3] = {};
        };

        using Layout = ConstraintLayout<
            ConstraintBlock<NONE, 3, &Cache::pointLambda>,
            ConstraintBlock<ANGULAR, 3, &Cache::angularLambda>>;

        using Base = JointImpl<FixedJoint, Layout, Cache>;
    };

    struct PHYSECS_API FixedJoint final : FixedJointDef::Base {
        static void makeConstraints(const JointWorldSpaceData& worldSpaceData, void* additionalData, Constraint1DWriter& constraints);
        using FixedJointDef::Base::Base;
    };
}

