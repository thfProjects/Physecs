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

#ifndef PHYSECS_FORCE_INLINE
    #if defined(_MSC_VER)
        #define PHYSECS_FORCE_INLINE __forceinline
    #else
        #define PHYSECS_FORCE_INLINE __attribute__((always_inline)) inline
    #endif
#endif

#ifndef PHYSECS_FORCE_INLINE_LAMBDA
    #if defined(_MSC_VER)
        #define PHYSECS_FORCE_INLINE_LAMBDA [[msvc::forceinline]]
    #else
        #define PHYSECS_FORCE_INLINE_LAMBDA __attribute__((always_inline))
    #endif
#endif
