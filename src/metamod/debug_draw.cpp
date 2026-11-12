#include "debug_draw.h"

#include <vector>
#include <cmath>
#include <extdll.h>
#include <meta_api.h>
#include <util.h> // PRECACHE_MODEL/MESSAGE_BEGIN/WRITE_*/MESSAGE_END/INDEXENT

#include "core/goldnav_module.h"

namespace goldnav {

namespace {
int g_beamSpriteIndex = 0;
float g_nextDrawTime = 0.0f;

// Лучей на клиенте помещается ограниченное число, а сетка мелкая — рисуем только
// ближайшие к игроку рёбра и не чаще двух раз в секунду.
constexpr float kDrawInterval = 0.5f;
constexpr float kDrawRadius = 160.0f;
constexpr int kMaxGridBeams = 140;
constexpr int kMaxPathBeams = 60;

void DrawBeam(const Vector& start, const Vector& end, int r, int g, int b, int width)
{
    MESSAGE_BEGIN(MSG_BROADCAST, SVC_TEMPENTITY);
        WRITE_BYTE(TE_BEAMPOINTS);
        WRITE_COORD(start.x); WRITE_COORD(start.y); WRITE_COORD(start.z);
        WRITE_COORD(end.x); WRITE_COORD(end.y); WRITE_COORD(end.z);
        WRITE_SHORT(g_beamSpriteIndex);
        WRITE_BYTE(0);   // startframe
        WRITE_BYTE(0);   // framerate
        WRITE_BYTE(static_cast<int>(kDrawInterval * 10.0f) + 1); // life, 0.1 c
        WRITE_BYTE(width);
        WRITE_BYTE(0);   // noise
        WRITE_BYTE(r);
        WRITE_BYTE(g);
        WRITE_BYTE(b);
        WRITE_BYTE(200); // brightness
        WRITE_BYTE(0);   // scroll speed
    MESSAGE_END();
}
} // namespace

void DebugDraw_Precache()
{
    // Стандартный HL-спрайт луча, присутствует в любой валидной установке HLDS.
    g_beamSpriteIndex = PRECACHE_MODEL("sprites/laserbeam.spr");
    g_nextDrawTime = 0.0f;
}

void DebugDraw_Tick()
{
    if (!gpGlobals || gpGlobals->time < g_nextDrawTime)
    {
        return;
    }
    g_nextDrawTime = gpGlobals->time + kDrawInterval;

    GoldNavModule& module = GoldNavModule::Instance();

    // Пути идущих entity — жёлтым, чуть над полом.
    std::vector<std::vector<Vec3>> paths;
    module.CollectActivePaths(paths);
    int pathBeams = 0;
    for (const std::vector<Vec3>& path : paths)
    {
        for (size_t i = 1; i < path.size() && pathBeams < kMaxPathBeams; ++i)
        {
            DrawBeam(Vector(path[i - 1].x, path[i - 1].y, path[i - 1].z + 6.0f),
                     Vector(path[i].x, path[i].y, path[i].z + 6.0f), 255, 220, 0, 6);
            ++pathBeams;
        }
    }

    if (!module.IsReady())
    {
        return;
    }

    edict_t* player = nullptr;
    for (int i = 1; i <= gpGlobals->maxClients; ++i)
    {
        edict_t* candidate = INDEXENT(i);
        if (candidate && !candidate->free && (candidate->v.flags & FL_CLIENT) != 0)
        {
            player = candidate;
            break;
        }
    }
    if (!player)
    {
        return;
    }

    // Сетка основного слоя вокруг игрока: зелёным — обычные шаги, красным — спрыгивания.
    const NavGrid& grid = module.Layer(NavLayer::Medium).grid;
    const Vec3 origin(player->v.origin.x, player->v.origin.y, player->v.origin.z);

    int cx = 0;
    int cy = 0;
    grid.WorldToCell(origin, cx, cy);
    const int radius = static_cast<int>(kDrawRadius / grid.CellSize());

    int beams = 0;
    for (int gx = cx - radius; gx <= cx + radius && beams < kMaxGridBeams; ++gx)
    {
        for (int gy = cy - radius; gy <= cy + radius && beams < kMaxGridBeams; ++gy)
        {
            for (uint32_t id = grid.ColumnHead(gx, gy); id != kInvalidNodeId; id = grid.Node(id).nextInColumn)
            {
                const NavNode& node = grid.Node(id);
                if (std::fabs(node.pos.z - origin.z) > 96.0f)
                {
                    continue;
                }

                // Только +X и +Y — каждое ребро один раз.
                static const int kDirs[] = { 0, 2 };
                for (int dir : kDirs)
                {
                    if (node.edgeType[dir] == kEdgeNone || beams >= kMaxGridBeams)
                    {
                        continue;
                    }
                    const NavNode& other = grid.Node(node.edgeTarget[dir]);
                    const bool drop = node.edgeType[dir] == kEdgeDrop;
                    const bool jump = node.edgeType[dir] == kEdgeJump;
                    DrawBeam(Vector(node.pos.x, node.pos.y, node.pos.z + 2.0f),
                             Vector(other.pos.x, other.pos.y, other.pos.z + 2.0f),
                             drop ? 255 : 0, (drop || jump) ? (jump ? 120 : 0) : 255, jump ? 255 : 0, 2);
                    ++beams;
                }
            }
        }
    }
}

} // namespace goldnav
