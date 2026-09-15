#pragma once

#if defined(_MSC_VER)
    #include <intrin.h>
#endif

namespace physecs
{
inline bool bitScanForward(unsigned long* index, unsigned long mask) {
#if defined(_MSC_VER)
        return _BitScanForward(index, mask) != 0;
#else
        if (mask == 0) {
            return false;
        }
        *index = static_cast<unsigned long>(__builtin_ctzl(mask));
        return true;
#endif
}
}