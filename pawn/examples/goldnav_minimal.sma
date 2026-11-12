/* GoldNav: минимальный пример.
 *
 * say /npc  — создаёт entity перед вами и отправляет её в точку, на которую вы смотрите.
 * Результат приходит в goldnav_move_finished.
 */
#include <amxmodx>
#include <fakemeta>
#include <goldnav>

new const Float:NPC_MINS[3] = { -16.0, -16.0, 0.0 }
new const Float:NPC_MAXS[3] = { 16.0, 16.0, 36.0 }   // ровно "приседающий" хулл движка

new g_npc

public plugin_precache()
{
    precache_model("models/chick.mdl")
}

public plugin_init()
{
    register_plugin("GoldNav Minimal", "1.0", "Hozon")
    register_clcmd("say /npc", "CmdNpc")
}

public CmdNpc(id)
{
    new Float:goal[3]
    if (!GetAimPoint(id, goal))
        return PLUGIN_HANDLED

    // Создаём entity, если её ещё нет.
    if (!pev_valid(g_npc))
    {
        g_npc = CreateNpc(id)
        if (!g_npc)
        {
            client_print(id, print_chat, "[Example] Не удалось создать entity.")
            return PLUGIN_HANDLED
        }
    }

    // Главный вызов: дальше GoldNav всё делает сам. Можно вызывать, даже если навмеш
    // ещё строится — запрос подождёт готовности.
    if (goldnav_move_to(g_npc, goal, 180.0))
        client_print(id, print_chat, "[Example] Пошла в точку %.0f %.0f %.0f", goal[0], goal[1], goal[2])
    else
        client_print(id, print_chat, "[Example] GoldNav отказал (entity невалидна?).")

    return PLUGIN_HANDLED
}

// Вызывается модулем, когда entity дошла или не смогла дойти.
public goldnav_move_finished(ent, result)
{
    if (ent != g_npc)
        return

    client_print(0, print_chat, "[Example] %s", result == GOLDNAV_STATE_ARRIVED ? "Дошла!" : "Не смогла дойти.")
}

CreateNpc(id)
{
    new ent = engfunc(EngFunc_CreateNamedEntity, engfunc(EngFunc_AllocString, "info_target"))
    if (!pev_valid(ent))
        return 0

    engfunc(EngFunc_SetModel, ent, "models/chick.mdl")
    engfunc(EngFunc_SetSize, ent, NPC_MINS, NPC_MAXS)   // после SetModel — он сбрасывает размер
    set_pev(ent, pev_classname, "example_npc")
    set_pev(ent, pev_solid, SOLID_BBOX)
    set_pev(ent, pev_movetype, MOVETYPE_PUSHSTEP)       // клиент плавно интерполирует движение

    new Float:origin[3]
    pev(id, pev_origin, origin)
    engfunc(EngFunc_SetOrigin, ent, origin)
    engfunc(EngFunc_DropToFloor, ent)
    return ent
}

// Точка, на которую смотрит игрок.
bool:GetAimPoint(id, Float:point[3])
{
    new Float:start[3], Float:viewOfs[3], Float:angles[3], Float:dir[3], Float:end[3]
    pev(id, pev_origin, start)
    pev(id, pev_view_ofs, viewOfs)
    pev(id, pev_v_angle, angles)
    angle_vector(angles, ANGLEVECTOR_FORWARD, dir)

    for (new i = 0; i < 3; i++)
    {
        start[i] += viewOfs[i]
        end[i] = start[i] + dir[i] * 4096.0
    }

    engfunc(EngFunc_TraceLine, start, end, IGNORE_MONSTERS, id, 0)
    get_tr2(0, TR_vecEndPos, point)
    return true
}
