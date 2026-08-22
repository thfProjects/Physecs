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

        using Layout = ConstraintLayout<
            ConstraintBlock<NONE, 3, &Cache::pointLambda>,
            ConstraintBlock<ANGULAR, 2, &Cache::angularLambda>,
            ConstraintBlock<ANGULAR | LIMITED, 1, &Cache::driveLambda, &Data::driveEnabled>>;

        using Base = JointImpl<RevoluteJoint, Layout, Cache, Data>;
    };

    PHYSECS_DECLARE_JOINT_IMPL(RevoluteJoint);

    struct PHYSECS_API RevoluteJoint final : RevoluteJointDef::Base {
        static void makeConstraints(const JointWorldSpaceData& worldSpaceData, void* additionalData, Constraint1DWriter& constraints);
        using RevoluteJointDef::Base::Base;

        void setDriveEnabled(bool enabled);
        void setDriveVelocity(float velocity);
        void setDriveMaxTorque(float maxTorque);
    };
}
