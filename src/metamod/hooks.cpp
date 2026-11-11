#include "hooks.h"

#include <extdll.h>
#include <meta_api.h>

#include "core/goldnav_module.h"
#include "core/config.h"
#include "metamod/debug_draw.h"

namespace goldnav {

void Hook_ServerActivate_Post(edict_t* /*pEdictList*/, int /*edictCount*/, int /*clientMax*/)
{
    // POST: мир и все static/point-энтити карты уже заспавнены движком к этому
    // моменту — безопасно начинать семплинг геометрии.
    GoldNavModule::Instance().OnServerActivate();
    DebugDraw_Precache();
}

void Hook_ServerDeactivate_Post()
{
    GoldNavModule::Instance().OnServerDeactivate();
}

void Hook_StartFrame_Post()
{
    const float frameTime = gpGlobals ? gpGlobals->frametime : 0.0f;
    GoldNavModule::Instance().OnStartFrame(frameTime);

    if (ConfigDebugDrawEnabled())
    {
        DebugDraw_Tick();
    }
}

} // namespace goldnav
