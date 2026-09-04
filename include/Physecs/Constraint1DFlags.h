#pragma once

namespace physecs {
    enum ConstraintFlags {
        NONE = 0,
        SOFT = 1,
        ANGULAR = 1 << 1,
        LIMITED = 1 << 2
    };
}

