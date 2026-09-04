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

        // angular blocks come first so orthogonalization never gives them a linear part
        using Layout = ConstraintLayout<
            ConstraintBlock<ANGULAR, 3, &Cache::angularLambda>,
            ConstraintBlock<NONE, 3, &Cache::pointLambda>>;

        using Base = JointImpl<FixedJoint, Layout, Cache, Data>;
    };

    PHYSECS_DECLARE_JOINT_IMPL(FixedJoint);

    struct PHYSECS_API FixedJoint final : FixedJointDef::Base {
        static void makeConstraints(const JointWorldSpaceData& worldSpaceData, void* additionalData, Constraint1DDescriptor* constraintRows);
        using FixedJointDef::Base::Base;
    };
}

