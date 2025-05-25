#pragma once

#ifdef PHYSECS_EXPORTS
    #if defined(_MSC_VER)
        #define PHYSECS_API __declspec(dllexport)
    #elif defined(__GNUC__)
        #define PHYSECS_API __attribute__((visibility("default")))
    #endif
#else
    #define PHYSECS_API
#endif
