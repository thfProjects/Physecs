#pragma once

#ifdef PHYSECS_EXPORTS
    #define PHYSECS_API __declspec(dllexport)
#else
    #define PHYSECS_API
#endif
