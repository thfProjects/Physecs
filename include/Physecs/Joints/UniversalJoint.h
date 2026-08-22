#pragma once
#include "Joint.h"

namespace physecs {
    struct UniversalJoint;

    struct UniversalJointDef {
        struct Cache {
            float pointLambda[3] = {};
            float angularLambda = 0;
        };

        using Layout = ConstraintLayout<
            ConstraintBlock<NONE, 3, &Cache::pointLambda>,
            ConstraintBlock<ANGULAR, 1, &Cache::angularLambda>>;

        using Base = JointImpl<UniversalJoint, Layout, Cache>;
    };

    struct PHYSECS_API UniversalJoint final : UniversalJointDef::Base {
        static void makeConstraints(const JointWorldSpaceData& worldSpaceData, void* additionalData, Constraint1DWriter& constraints);
        using UniversalJointDef::Base::Base;
    };
}
