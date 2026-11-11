/* GoldNav: тестовый плагин "курица".
 *
 * Спавнит по прицелу entity с моделью models/chick.mdl и по команде отправляет её в
 * точку, на которую вы смотрите — чтобы глазами проверить, как модуль GoldNav ведёт
 * entity: обходит стены, поднимается по ступенькам, огибает игроков.
 *
 * Команды (консоль клиента или чат):
 *   goldnav_chicken   | say /chicken  — заспавнить курицу там, куда смотрите
 *   goldnav_goto      | say /goto     — отправить своих кур в точку прицела
 *   goldnav_goto x y z                — ...или в явные координаты
 *   goldnav_follow    | say /follow   — куры идут за вами (повторно — перестают)
 *   goldnav_stop      | say /stop     — остановить
 *   goldnav_remove    | say /remove   — удалить своих кур
 *   goldnav_where     | say /where    — координаты точки прицела
 *
 * Квары:
 *   goldnav_chicken_speed 150   — скорость кур, юнитов/с
 *   goldnav_chicken_path 1      — рисовать найденный путь лучом (видит только отправивший)
 *   goldnav_chicken_admin 0     — 1: команды только для админов (флаг "d", ADMIN_BAN)
 *
 * Требует модули fakemeta и goldnav (+ goldnav_mm в Metamod).
 *
 * Автор: Hozon (Telegram: @AmxM0dX)
 */
#include <amxmodx>
#include <amxmisc>
#include <fakemeta>
#include <goldnav>

#define PLUGIN  "GoldNav Chicken"
#define VERSION "1.0.0-beta"
#define AUTHOR  "Hozon"

new const CHICKEN_CLASS[] = "goldnav_chicken"
new const CHICKEN_MODEL[] = "models/chick.mdl"
new const BEAM_SPRITE[]   = "sprites/laserbeam.spr"

// Курица маленькая, но bbox ровно под "приседающий" хулл движка (32x32x36): тогда
// движок сталкивает её с картой честно и по центру, без подмены размера и смещения
// (см. комментарий про хуллы в goldnav.inc).
new const Float:CHICKEN_MINS[3] = { -16.0, -16.0, 0.0 }
new const Float:CHICKEN_MAXS[3] = { 16.0, 16.0, 36.0 }

const MAX_CHICKENS_PER_PLAYER = 8
const MAX_PATH_POINTS = 128
const pev_chicken_owner = pev_iuser1   // индекс игрока-владельца
const pev_chicken_mark  = pev_iuser2   // метка "это наша курица"
const CHICKEN_MARK = 0x474E4156        // 'GNAV'

const Float:FOLLOW_INTERVAL = 0.4
const Float:FOLLOW_REPATH_DISTANCE = 72.0   // игрок отошёл дальше — ведём кур заново
const Float:FOLLOW_STOP_DISTANCE = 80.0     // ближе этого к игроку куры не лезут

new g_cvarSpeed, g_cvarDrawPath, g_cvarAdminOnly
new g_beamSprite
new bool:g_following[33]
new Float:g_followGoal[33][3]

public plugin_precache()
{
    precache_model(CHICKEN_MODEL)
    g_beamSprite = precache_model(BEAM_SPRITE)
}

public plugin_init()
{
    register_plugin(PLUGIN, VERSION, AUTHOR)

    register_clcmd("goldnav_chicken", "CmdSpawn")
    register_clcmd("goldnav_goto", "CmdGoto")
    register_clcmd("goldnav_follow", "CmdFollow")
    register_clcmd("goldnav_stop", "CmdStop")
    register_clcmd("goldnav_remove", "CmdRemove")
    register_clcmd("goldnav_where", "CmdWhere")

    register_clcmd("say /chicken", "CmdSpawn")
    register_clcmd("say /goto", "CmdGoto")
    register_clcmd("say /follow", "CmdFollow")
    register_clcmd("say /stop", "CmdStop")
    register_clcmd("say /remove", "CmdRemove")
    register_clcmd("say /where", "CmdWhere")

    g_cvarSpeed     = register_cvar("goldnav_chicken_speed", "150")
    g_cvarDrawPath  = register_cvar("goldnav_chicken_path", "1")
    g_cvarAdminOnly = register_cvar("goldnav_chicken_admin", "0")

    set_task(FOLLOW_INTERVAL, "TaskFollow", _, _, _, "b")
}

public client_disconnected(id)
{
    g_following[id] = false
    RemoveChickens(id)
}

// --- Команды ---

public CmdSpawn(id)
{
    if (!CanUse(id))
        return PLUGIN_HANDLED

    if (CountChickens(id) >= MAX_CHICKENS_PER_PLAYER)
    {
        client_print(id, print_chat, "[GoldNav] У вас уже %d кур - больше нельзя (goldnav_remove).", MAX_CHICKENS_PER_PLAYER)
        return PLUGIN_HANDLED
    }

    new Float:spot[3]
    if (!GetAimSpot(id, spot, 24.0))
    {
        client_print(id, print_chat, "[GoldNav] Не вижу, куда ставить курицу.")
        return PLUGIN_HANDLED
    }

    new ent = SpawnChicken(id, spot)
    if (!ent)
    {
        client_print(id, print_chat, "[GoldNav] Не удалось создать курицу (нет места?).")
        return PLUGIN_HANDLED
    }

    client_print(id, print_chat, "[GoldNav] Курица #%d на месте. Наведите прицел и введите goldnav_goto (или /goto).", ent)
    if (!goldnav_is_ready(GOLDNAV_HULL_DUCKING))
        client_print(id, print_chat, "[GoldNav] Навмеш карты еще строится - курица пойдет, как только он будет готов.")

    return PLUGIN_HANDLED
}

public CmdGoto(id)
{
    if (!CanUse(id))
        return PLUGIN_HANDLED

    new Float:goal[3]
    if (read_argc() >= 4)
    {
        // goldnav_goto x y z
        new arg[32]
        for (new i = 0; i < 3; i++)
        {
            read_argv(i + 1, arg, charsmax(arg))
            goal[i] = str_to_float(arg)
        }
    }
    else if (!GetAimSpot(id, goal, 4.0))
    {
        client_print(id, print_chat, "[GoldNav] Не вижу точку прицела.")
        return PLUGIN_HANDLED
    }

    g_following[id] = false

    new sent = SendChickens(id, goal, true)
    if (!sent)
        client_print(id, print_chat, "[GoldNav] У вас нет кур. Сначала goldnav_chicken (или /chicken).")
    else
        client_print(id, print_chat, "[GoldNav] Кур отправлено: %d -> (%.0f %.0f %.0f)", sent, goal[0], goal[1], goal[2])

    return PLUGIN_HANDLED
}

public CmdFollow(id)
{
    if (!CanUse(id))
        return PLUGIN_HANDLED

    if (!CountChickens(id))
    {
        client_print(id, print_chat, "[GoldNav] У вас нет кур. Сначала goldnav_chicken (или /chicken).")
        return PLUGIN_HANDLED
    }

    g_following[id] = !g_following[id]
    if (g_following[id])
    {
        g_followGoal[id][0] = g_followGoal[id][1] = g_followGoal[id][2] = 99999.0
        client_print(id, print_chat, "[GoldNav] Куры идут за вами. Повторите команду, чтобы отпустить.")
    }
    else
    {
        StopChickens(id)
        client_print(id, print_chat, "[GoldNav] Куры больше не идут за вами.")
    }
    return PLUGIN_HANDLED
}

public CmdStop(id)
{
    if (!CanUse(id))
        return PLUGIN_HANDLED

    g_following[id] = false
    client_print(id, print_chat, "[GoldNav] Остановлено кур: %d", StopChickens(id))
    return PLUGIN_HANDLED
}

public CmdRemove(id)
{
    if (!CanUse(id))
        return PLUGIN_HANDLED

    g_following[id] = false
    client_print(id, print_chat, "[GoldNav] Удалено кур: %d", RemoveChickens(id))
    return PLUGIN_HANDLED
}

public CmdWhere(id)
{
    new Float:spot[3]
    if (GetAimSpot(id, spot, 0.0))
    {
        client_print(id, print_chat, "[GoldNav] Прицел: %.1f %.1f %.1f", spot[0], spot[1], spot[2])
        console_print(id, "[GoldNav] aim: %.1f %.1f %.1f", spot[0], spot[1], spot[2])
    }
    return PLUGIN_HANDLED
}

// --- События GoldNav ---

public goldnav_move_finished(ent, result)
{
    if (!IsChicken(ent))
        return

    set_pev(ent, pev_framerate, 1.0)

    new owner = pev(ent, pev_chicken_owner)
    if (!is_user_connected(owner) || g_following[owner])
        return

    if (result == GOLDNAV_STATE_ARRIVED)
        client_print(owner, print_chat, "[GoldNav] Курица #%d дошла.", ent)
    else
        client_print(owner, print_chat, "[GoldNav] Курица #%d не смогла дойти (нет пути или застряла).", ent)
}

// --- Следование за игроком ---

public TaskFollow()
{
    new Float:origin[3]
    for (new id = 1; id <= 32; id++)
    {
        if (!g_following[id] || !is_user_alive(id))
            continue

        pev(id, pev_origin, origin)

        // Игрок почти не сдвинулся — куры уже идут куда надо, не дёргаем их.
        if (get_distance_f(origin, g_followGoal[id]) < FOLLOW_REPATH_DISTANCE)
            continue

        g_followGoal[id][0] = origin[0]
        g_followGoal[id][1] = origin[1]
        g_followGoal[id][2] = origin[2]
        SendChickens(id, origin, false)
    }
}

// --- Работа с курами ---

SpawnChicken(owner, const Float:spot[3])
{
    new ent = engfunc(EngFunc_CreateNamedEntity, engfunc(EngFunc_AllocString, "info_target"))
    if (!pev_valid(ent))
        return 0

    set_pev(ent, pev_classname, CHICKEN_CLASS)
    engfunc(EngFunc_SetModel, ent, CHICKEN_MODEL)
    engfunc(EngFunc_SetSize, ent, CHICKEN_MINS, CHICKEN_MAXS)   // после SetModel: тот сбрасывает размер

    set_pev(ent, pev_solid, SOLID_BBOX)
    // PUSHSTEP: для него клиент сам плавно интерполирует перемещение между обновлениями.
    set_pev(ent, pev_movetype, MOVETYPE_PUSHSTEP)
    set_pev(ent, pev_takedamage, DAMAGE_NO)
    set_pev(ent, pev_chicken_owner, owner)
    set_pev(ent, pev_chicken_mark, CHICKEN_MARK)

    // У chick.mdl одна анимация ("peck", зацикленная) — её и крутим.
    set_pev(ent, pev_sequence, 0)
    set_pev(ent, pev_frame, 0.0)
    set_pev(ent, pev_framerate, 1.0)
    set_pev(ent, pev_animtime, get_gametime())

    // Лицом от игрока.
    new Float:angles[3]
    pev(owner, pev_v_angle, angles)
    angles[0] = 0.0
    angles[2] = 0.0
    set_pev(ent, pev_angles, angles)

    engfunc(EngFunc_SetOrigin, ent, spot)

    // Точка прицела может быть на стене или в воздухе — ставим на пол под ней.
    if (engfunc(EngFunc_DropToFloor, ent) == -1)
    {
        engfunc(EngFunc_RemoveEntity, ent)   // место занято геометрией
        return 0
    }
    return ent
}

SendChickens(owner, const Float:goal[3], bool:drawPath)
{
    new Float:speed = get_pcvar_float(g_cvarSpeed)
    new flags = 0   // GoldNav сам поворачивает курицу по ходу движения

    new sent = 0, ent = -1
    new Float:origin[3]
    while ((ent = engfunc(EngFunc_FindEntityByString, ent, "classname", CHICKEN_CLASS)) > 0)
    {
        if (!IsChicken(ent) || pev(ent, pev_chicken_owner) != owner)
            continue

        pev(ent, pev_origin, origin)

        // В режиме "за мной" кур, которые уже рядом, не трогаем — иначе толпились бы у ног.
        if (!drawPath && get_distance_f(origin, goal) < FOLLOW_STOP_DISTANCE)
        {
            goldnav_stop_following(ent)
            set_pev(ent, pev_framerate, 1.0)
            continue
        }

        if (!goldnav_move_to(ent, goal, speed, GOLDNAV_HULL_AUTO, flags))
            continue

        set_pev(ent, pev_framerate, 2.0)   // "семенит" быстрее, пока идёт
        sent++

        if (drawPath && get_pcvar_num(g_cvarDrawPath))
            DrawPath(owner, origin, goal)
    }
    return sent
}

StopChickens(owner)
{
    new stopped = 0, ent = -1
    while ((ent = engfunc(EngFunc_FindEntityByString, ent, "classname", CHICKEN_CLASS)) > 0)
    {
        if (!IsChicken(ent) || pev(ent, pev_chicken_owner) != owner)
            continue
        goldnav_stop_following(ent)
        set_pev(ent, pev_framerate, 1.0)
        stopped++
    }
    return stopped
}

RemoveChickens(owner)
{
    // Сначала собираем, потом удаляем: удаление во время FindEntityByString сбило бы обход.
    new list[64], count = 0, ent = -1
    while (count < sizeof(list) && (ent = engfunc(EngFunc_FindEntityByString, ent, "classname", CHICKEN_CLASS)) > 0)
    {
        if (IsChicken(ent) && pev(ent, pev_chicken_owner) == owner)
            list[count++] = ent
    }

    for (new i = 0; i < count; i++)
    {
        goldnav_stop_following(list[i])
        engfunc(EngFunc_RemoveEntity, list[i])
    }
    return count
}

CountChickens(owner)
{
    new count = 0, ent = -1
    while ((ent = engfunc(EngFunc_FindEntityByString, ent, "classname", CHICKEN_CLASS)) > 0)
    {
        if (IsChicken(ent) && pev(ent, pev_chicken_owner) == owner)
            count++
    }
    return count
}

bool:IsChicken(ent)
{
    return pev_valid(ent) && pev(ent, pev_chicken_mark) == CHICKEN_MARK
}

// --- Вспомогательное ---

bool:CanUse(id)
{
    if (!is_user_connected(id))
        return false

    if (get_pcvar_num(g_cvarAdminOnly) && !(get_user_flags(id) & ADMIN_BAN))
    {
        client_print(id, print_chat, "[GoldNav] Команда только для админов (goldnav_chicken_admin 0 - разрешить всем).")
        return false
    }
    return true
}

// Точка, на которую смотрит игрок, отодвинутая от поверхности на pushOut юнитов
// вдоль нормали (чтобы не оказаться "в стене").
bool:GetAimSpot(id, Float:spot[3], Float:pushOut)
{
    new Float:start[3], Float:viewOfs[3], Float:angles[3], Float:dir[3], Float:end[3]
    pev(id, pev_origin, start)
    pev(id, pev_view_ofs, viewOfs)
    start[0] += viewOfs[0]
    start[1] += viewOfs[1]
    start[2] += viewOfs[2]

    pev(id, pev_v_angle, angles)
    angle_vector(angles, ANGLEVECTOR_FORWARD, dir)
    end[0] = start[0] + dir[0] * 8192.0
    end[1] = start[1] + dir[1] * 8192.0
    end[2] = start[2] + dir[2] * 8192.0

    new tr = create_tr2()
    engfunc(EngFunc_TraceLine, start, end, IGNORE_MONSTERS, id, tr)

    new Float:fraction
    get_tr2(tr, TR_flFraction, fraction)
    if (fraction >= 1.0)
    {
        free_tr2(tr)
        return false
    }

    new Float:normal[3]
    get_tr2(tr, TR_vecEndPos, spot)
    get_tr2(tr, TR_vecPlaneNormal, normal)
    free_tr2(tr)

    spot[0] += normal[0] * pushOut
    spot[1] += normal[1] * pushOut
    spot[2] += normal[2] * pushOut
    return true
}

// Показывает игроку путь, который GoldNav нашёл от from до goal.
DrawPath(id, const Float:from[3], const Float:goal[3])
{
    static Float:path[MAX_PATH_POINTS * 3]
    new count
    if (!goldnav_find_path(from, goal, path, MAX_PATH_POINTS, count, GOLDNAV_HULL_DUCKING) || count <= 0)
        return

    new Float:prev[3], Float:point[3]
    prev[0] = from[0]
    prev[1] = from[1]
    prev[2] = from[2]

    for (new i = 0; i < count; i++)
    {
        point[0] = path[i * 3]
        point[1] = path[i * 3 + 1]
        point[2] = path[i * 3 + 2] + 4.0

        message_begin(MSG_ONE_UNRELIABLE, SVC_TEMPENTITY, _, id)
        write_byte(TE_BEAMPOINTS)
        engfunc(EngFunc_WriteCoord, prev[0])
        engfunc(EngFunc_WriteCoord, prev[1])
        engfunc(EngFunc_WriteCoord, prev[2])
        engfunc(EngFunc_WriteCoord, point[0])
        engfunc(EngFunc_WriteCoord, point[1])
        engfunc(EngFunc_WriteCoord, point[2])
        write_short(g_beamSprite)
        write_byte(0)     // стартовый кадр
        write_byte(0)     // частота кадров
        write_byte(80)    // время жизни, 0.1 с
        write_byte(8)     // ширина
        write_byte(0)     // шум
        write_byte(255)   // r
        write_byte(220)   // g
        write_byte(0)     // b
        write_byte(220)   // яркость
        write_byte(0)     // прокрутка
        message_end()

        prev[0] = point[0]
        prev[1] = point[1]
        prev[2] = point[2]
    }
}
