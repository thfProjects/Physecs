#pragma once

#ifdef PHYSECS_PROFILE
#include <tracy/Tracy.hpp>
#include <tracy/TracyC.h>

#define PhysecsFrameMarkStart(frameName) FrameMarkStart(frameName)
#define PhysecsFrameMarkEnd(frameName) FrameMarkEnd(frameName)
#define PhysecsZoneScoped ZoneScoped
#define PhysecsZoneScopedN(name) ZoneScopedN(name)
#define PhysecsZoneN(ctx, name, active) TracyCZoneN(ctx, name, active);
#define PhysecsZoneEnd(ctx) TracyCZoneEnd(ctx);
#else
#define PhysecsFrameMarkStart(frameName)
#define PhysecsFrameMarkEnd(frameName)
#define PhysecsZoneScoped
#define PhysecsZoneScopedN(name)
#define PhysecsZoneN(ctx, name, active)
#define PhysecsZoneEnd(ctx)
#endif
