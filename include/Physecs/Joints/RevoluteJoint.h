#pragma once
#include "Joint.h"

namespace physecs {
    struct RevoluteJoint;

    struct RevoluteJointDef {
        struct Data {
            bool driveEnabled = false;
            float driveVelocity = 0;
            float driveMaxTorque = 0;
        };

        struct Cache {
            float pointLambda[3] = {};
            float angularLambda[2] = {};
            float driveLambda = 0;
        };

        // angular blocks come first so orthogonalization never gives them a linear part
        using Layout = ConstraintLayout<
            ConstraintBlock<ANGULAR, 2, &Cache::angularLambda>,
            ConstraintBlock<NONE, 3, &Cache::pointLambda>,
            ConstraintBlock<ANGULAR | LIMITED, 1, &Cache::driveLambda, &Data::driveEnabled>>;

        using Base = JointImpl<RevoluteJoint, Layout, Cache, Data>;
    };

    PHYSECS_DECLARE_JOINT_IMPL(RevoluteJoint);

    struct PHYSECS_API RevoluteJoint final : RevoluteJointDef::Base {
        static void makeConstraints(const JointWorldSpaceData& worldSpaceData, void* additionalData, Constraint1DDescriptor* constraintRows);
        using RevoluteJointDef::Base::Base;

        void setDriveEnabled(bool enabled);
        void setDriveVelocity(float velocity);
        void setDriveMaxTorque(float maxTorque);
    };
}
