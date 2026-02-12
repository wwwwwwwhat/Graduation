#ifndef THESIS_RCSPP_NAVMESH_CONVERTER_H
#define THESIS_RCSPP_NAVMESH_CONVERTER_H

#include "graph/Roadmap.h"
#include <string>
#include <unordered_map>
#include <functional>

// Forward declarations from Detour
class dtNavMesh;
struct dtMeshTile;
struct dtPoly;

namespace rcspp {

// Terrain type enumeration
enum TerrainType : unsigned char {
    TERRAIN_GROUND = 0,   // Normal ground
    TERRAIN_GRASS  = 1,   // Grass
    TERRAIN_SAND   = 2,   // Sand (high stamina)
    TERRAIN_WATER  = 3,   // Water (high AP)
    TERRAIN_ROCK   = 4,   // Rocky terrain
    TERRAIN_MUD    = 5,   // Mud
};

// Resource cost configuration per terrain type
struct TerrainConfig {
    float staminaMultiplier = 1.0f;
    float apMultiplier = 1.0f;
};

// Enhanced NavMesh to Graph converter
class NavMeshConverter {
public:
    NavMeshConverter();
    ~NavMeshConverter();

    // Set terrain configuration
    void SetTerrainConfig(unsigned char terrainType, const TerrainConfig& config);

    // Convert NavMesh to Roadmap (direct in-memory graph)
    // costDim: 1 (distance) + number of resources
    // resourceTypes: number of resource types (default 2: stamina, AP)
    Roadmap ConvertToRoadmap(const dtNavMesh* navMesh, int resourceTypes = 2);

    // Export to DIMACS format files (for compatibility with public_erca)
    bool ExportToDIMACS(const Roadmap& graph,
                        const std::string& costFile,
                        const std::vector<std::string>& resFiles);

    // Get polygon count from last conversion
    int GetPolygonCount() const { return polyCount_; }
    int GetEdgeCount() const { return edgeCount_; }

private:
    void computePolygonCenter(const dtMeshTile* tile, const dtPoly* poly, float* center);
    float distance(const float* a, const float* b);
    float getStaminaCost(unsigned char terrainType, float dist);
    float getAPCost(unsigned char terrainType, float dist);

    std::unordered_map<unsigned char, TerrainConfig> terrainConfigs_;
    int polyCount_ = 0;
    int edgeCount_ = 0;
};

} // namespace rcspp

#endif // THESIS_RCSPP_NAVMESH_CONVERTER_H
