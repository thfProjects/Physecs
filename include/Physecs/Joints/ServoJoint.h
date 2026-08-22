#pragma once
#include "Joint.h"

namespace physecs {
    struct ServoJoint;

    struct ServoJointDef {
        struct Data {
            float targetAngle = 0;
            float driveStiffness = 30;
            float driveDamping = 1;
        };

        struct Cache {
            float pointLambda[3] = {};
            float angularLambda[2] = {};
        };

        using Layout = ConstraintLayout<
            ConstraintBlock<NONE, 3, &Cache::pointLambda>,
            ConstraintBlock<ANGULAR, 2, &Cache::angularLambda>,
            ConstraintBlock<ANGULAR | SOFT>>;

        using Base = JointImpl<ServoJoint, Layout, Cache, Data>;
    };

    struct PHYSECS_API ServoJoint final : ServoJointDef::Base {
        static void makeConstraints(const JointWorldSpaceData& worldSpaceData, void* additionalData, Constraint1DWriter& constraints);
        using ServoJointDef::Base::Base;

        void setTargetAngle(float angle);
        void setDriveStiffness(float stiffness);
        void setDriveDamping(float damping);
    };
}
