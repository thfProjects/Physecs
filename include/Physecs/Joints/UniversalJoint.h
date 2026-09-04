#pragma once
#include "Joint.h"

namespace physecs {
    struct UniversalJoint;

    struct UniversalJointDef {
        struct Data {};

        struct Cache {
            float pointLambda[3] = {};
            float angularLambda = 0;
        };

        // angular blocks come first so orthogonalization never gives them a linear part
        using Layout = ConstraintLayout<
            ConstraintBlock<ANGULAR, 1, &Cache::angularLambda>,
            ConstraintBlock<NONE, 3, &Cache::pointLambda>>;

        using Base = JointImpl<UniversalJoint, Layout, Cache, Data>;
    };

    PHYSECS_DECLARE_JOINT_IMPL(UniversalJoint);

    struct PHYSECS_API UniversalJoint final : UniversalJointDef::Base {
        static void makeConstraints(const JointWorldSpaceData& worldSpaceData, void* additionalData, Constraint1DDescriptor* constraintRows);
        using UniversalJointDef::Base::Base;
    };
}
