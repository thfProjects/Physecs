#pragma once

#include <vector>
#include "ContactManifold.h"

namespace physecs {
    void clusterContactManifolds(std::vector<ContactManifold>& manifolds);
}
