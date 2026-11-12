#include "admin_commands.h"

#include <cstdlib>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <extdll.h>
#include <meta_api.h>
#include <util.h> // CMD_ARGC/CMD_ARGV

#include "core/goldnav_module.h"
#include "core/logger.h"

namespace goldnav {

namespace {

void Print(const char* fmt, ...)
{
    char line[512];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    SERVER_PRINT(line);
}

void Cmd_GoldNavRebuild()
{
    GoldNavModule::Instance().RequestRebuild();
}

void Cmd_GoldNavStatus()
{
    GoldNavModule& module = GoldNavModule::Instance();

    static const char* const kLayerNames[] = { "point", "32x32 (ducking/standing)", "64x64 (large)" };

    Print("[GoldNav] status: navReady=%d, entities being moved: %u\n",
          module.IsReady() ? 1 : 0, static_cast<unsigned>(module.AgentCount()));

    Print("  dynamic objects: %u tracked, %u standing boxes in navmesh\n",
          static_cast<unsigned>(module.TrackedObjectCount()), static_cast<unsigned>(module.ObstacleCount()));

    for (size_t i = 0; i < static_cast<size_t>(NavLayer::Count); ++i)
    {
        const NavLayerData& layer = module.Layer(static_cast<NavLayer>(i));
        if (!layer.started)
        {
            Print("  layer %s: not built (builds on first use)\n", kLayerNames[i]);
            continue;
        }

        Print("  layer %s: %s, cell=%.0f, nodes=%u, columns=%u, expanded=%u, queued=%u%s\n",
              kLayerNames[i], layer.initialDone ? "ready" : "GENERATING", layer.grid.CellSize(),
              static_cast<unsigned>(layer.grid.NodeCount()), static_cast<unsigned>(layer.grid.ColumnCount()),
              static_cast<unsigned>(layer.generator.ExpandedNodes()),
              static_cast<unsigned>(layer.generator.PendingNodes()),
              layer.generator.HitNodeLimit() ? " [node limit reached]" : "");
    }
}

void Cmd_GoldNavFindPath()
{
    // goldnav_findpath x1 y1 z1 x2 y2 z2 [hull]
    if (CMD_ARGC() < 7)
    {
        Print("[GoldNav] usage: goldnav_findpath x1 y1 z1 x2 y2 z2 [hull=2]  (hull: 0 point, 1 ducking, 2 standing, 3 large)\n");
        return;
    }

    const Vec3 from(
        static_cast<float>(std::atof(CMD_ARGV(1))),
        static_cast<float>(std::atof(CMD_ARGV(2))),
        static_cast<float>(std::atof(CMD_ARGV(3))));
    const Vec3 to(
        static_cast<float>(std::atof(CMD_ARGV(4))),
        static_cast<float>(std::atof(CMD_ARGV(5))),
        static_cast<float>(std::atof(CMD_ARGV(6))));
    const int hull = (CMD_ARGC() >= 8) ? std::atoi(CMD_ARGV(7)) : 2;

    std::vector<Vec3> path;
    const bool found = GoldNavModule::Instance().FindPath(from, to, hull, path);

    if (!found)
    {
        Print("[GoldNav] findpath: NOT FOUND (navmesh not ready or unreachable)\n");
        return;
    }

    float length = 0.0f;
    Vec3 prev = from;
    for (const Vec3& p : path)
    {
        length += Distance2D(prev, p);
        prev = p;
    }

    Print("[GoldNav] findpath: found, %u waypoints, length %.0f:\n", static_cast<unsigned>(path.size()), length);
    for (size_t i = 0; i < path.size(); ++i)
    {
        Print("  [%u] (%.1f, %.1f, %.1f)\n", static_cast<unsigned>(i), path[i].x, path[i].y, path[i].z);
    }
}

// goldnav_dump <file> [layer=1] — выгружает узлы слоя в бинарный файл (в папке игры)
// для внешнего просмотра/отладки. Формат: "GNAV1", float cellSize, u32 count, затем
// по узлу: float x,y,z; u8 variantMask, flags, wallDist[2]; u8 edgeType[8]; u32 edgeTarget[8].
void Cmd_GoldNavDump()
{
    if (CMD_ARGC() < 2)
    {
        Print("[GoldNav] usage: goldnav_dump <file> [layer=1]  (layer: 0 point, 1 32x32, 2 64x64)\n");
        return;
    }

    const int layerIndex = (CMD_ARGC() >= 3) ? std::atoi(CMD_ARGV(2)) : 1;
    if (layerIndex < 0 || layerIndex >= static_cast<int>(NavLayer::Count))
    {
        Print("[GoldNav] dump: bad layer\n");
        return;
    }

    const NavLayerData& layer = GoldNavModule::Instance().Layer(static_cast<NavLayer>(layerIndex));
    if (!layer.started)
    {
        Print("[GoldNav] dump: layer is not built\n");
        return;
    }

    char gameDir[256] = "";
    GET_GAME_DIR(gameDir);
    const std::string path = std::string(gameDir) + "/" + CMD_ARGV(1);

    FILE* file = std::fopen(path.c_str(), "wb");
    if (!file)
    {
        Print("[GoldNav] dump: cannot open %s\n", path.c_str());
        return;
    }

    const NavGrid& grid = layer.grid;
    const float cellSize = grid.CellSize();
    const uint32_t count = static_cast<uint32_t>(grid.NodeCount());

    std::fwrite("GNAV1", 1, 5, file);
    std::fwrite(&cellSize, sizeof(cellSize), 1, file);
    std::fwrite(&count, sizeof(count), 1, file);

    for (uint32_t id = 0; id < count; ++id)
    {
        const NavNode& node = grid.Node(id);
        const float pos[3] = { node.pos.x, node.pos.y, node.pos.z };
        const uint8_t meta[4] = { node.variantMask, node.flags, node.wallDist[0], node.wallDist[1] };
        std::fwrite(pos, sizeof(pos), 1, file);
        std::fwrite(meta, sizeof(meta), 1, file);
        std::fwrite(node.edgeType, 1, kNavDirCount, file);
        std::fwrite(node.edgeTarget, sizeof(uint32_t), kNavDirCount, file);
    }

    std::fclose(file);
    Print("[GoldNav] dump: %u nodes written to %s\n", count, path.c_str());
}

} // namespace

void RegisterAdminCommands()
{
    REG_SVR_COMMAND("goldnav_rebuild", Cmd_GoldNavRebuild);
    REG_SVR_COMMAND("goldnav_status", Cmd_GoldNavStatus);
    REG_SVR_COMMAND("goldnav_findpath", Cmd_GoldNavFindPath);
    REG_SVR_COMMAND("goldnav_dump", Cmd_GoldNavDump);
}

} // namespace goldnav
