#pragma once
#include "Joint.h"

namespace physecs {
    struct SphericalJoint;

    struct SphericalJointDef {
        struct Data {};

        struct Cache {
            float pointLambda[3] = {};
        };

        using Layout = ConstraintLayout<
            ConstraintBlock<NONE, 3, &Cache::pointLambda>>;

        using Base = JointImpl<SphericalJoint, Layout, Cache, Data>;
    };

    PHYSECS_DECLARE_JOINT_IMPL(SphericalJoint);

    struct PHYSECS_API SphericalJoint final : SphericalJointDef::Base {
        static void makeConstraints(const JointWorldSpaceData& worldSpaceData, void* additionalData, Constraint1DDescriptor* constraintRows);
        using SphericalJointDef::Base::Base;
    };
}
