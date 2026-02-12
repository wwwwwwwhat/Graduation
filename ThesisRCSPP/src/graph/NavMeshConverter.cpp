#include "graph/NavMeshConverter.h"

// Only compile if Detour headers available
#ifdef BUILD_NAVMESH
#include "DetourNavMesh.h"
#endif

#include <cmath>
#include <fstream>
#include <sstream>

namespace rcspp {

NavMeshConverter::NavMeshConverter() {
    // Default terrain configs
    terrainConfigs_[TERRAIN_GROUND] = {1.0f, 1.0f};
    terrainConfigs_[TERRAIN_GRASS]  = {1.2f, 1.0f};
    terrainConfigs_[TERRAIN_SAND]   = {2.0f, 1.5f};
    terrainConfigs_[TERRAIN_WATER]  = {1.5f, 2.5f};
    terrainConfigs_[TERRAIN_ROCK]   = {1.8f, 2.0f};
    terrainConfigs_[TERRAIN_MUD]    = {2.5f, 1.8f};
}

NavMeshConverter::~NavMeshConverter() {}

void NavMeshConverter::SetTerrainConfig(unsigned char terrainType, const TerrainConfig& config) {
    terrainConfigs_[terrainType] = config;
}

float NavMeshConverter::distance(const float* a, const float* b) {
    float dx = a[0] - b[0];
    float dy = a[1] - b[1];
    float dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

float NavMeshConverter::getStaminaCost(unsigned char terrainType, float dist) {
    float mult = 1.0f;
    if (terrainConfigs_.count(terrainType)) {
        mult = terrainConfigs_[terrainType].staminaMultiplier;
    }
    return dist * mult;
}

float NavMeshConverter::getAPCost(unsigned char terrainType, float dist) {
    float mult = 1.0f;
    if (terrainConfigs_.count(terrainType)) {
        mult = terrainConfigs_[terrainType].apMultiplier;
    }
    return dist * mult;
}

#ifdef BUILD_NAVMESH
void NavMeshConverter::computePolygonCenter(const dtMeshTile* tile, const dtPoly* poly, float* center) {
    center[0] = center[1] = center[2] = 0;
    for (int i = 0; i < poly->vertCount; i++) {
        const float* v = &tile->verts[poly->verts[i] * 3];
        center[0] += v[0];
        center[1] += v[1];
        center[2] += v[2];
    }
    float inv = 1.0f / poly->vertCount;
    center[0] *= inv;
    center[1] *= inv;
    center[2] *= inv;
}

Roadmap NavMeshConverter::ConvertToRoadmap(const dtNavMesh* navMesh, int resourceTypes) {
    size_t costDim = 1 + resourceTypes; // primary cost + resources
    Roadmap graph;

    // First pass: collect polygon centers
    struct PolyInfo {
        float center[3];
        unsigned char terrain;
    };
    std::unordered_map<long, PolyInfo> polyInfos;

    long nodeId = 1; // 1-based for DIMACS compatibility
    // Map: (tileIdx, polyIdx) -> nodeId
    std::unordered_map<unsigned int, long> refToNode;

    for (int i = 0; i < navMesh->getMaxTiles(); i++) {
        const dtMeshTile* tile = navMesh->getTile(i);
        if (!tile || !tile->header) continue;

        for (int j = 0; j < tile->header->polyCount; j++) {
            const dtPoly* poly = &tile->polys[j];
            if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) continue;

            dtPolyRef ref = navMesh->encodePolyId(tile->salt, i, j);

            PolyInfo info;
            computePolygonCenter(tile, poly, info.center);
            info.terrain = poly->getArea();

            polyInfos[nodeId] = info;
            refToNode[ref] = nodeId;
            graph.AddNode(nodeId);
            nodeId++;
        }
    }

    polyCount_ = static_cast<int>(polyInfos.size());
    graph.Init(polyCount_, costDim);

    // Re-add nodes after Init
    for (auto& [id, info] : polyInfos) {
        graph.AddNode(id);
    }

    // Second pass: build edges
    edgeCount_ = 0;
    for (int i = 0; i < navMesh->getMaxTiles(); i++) {
        const dtMeshTile* tile = navMesh->getTile(i);
        if (!tile || !tile->header) continue;

        for (int j = 0; j < tile->header->polyCount; j++) {
            const dtPoly* poly = &tile->polys[j];
            if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) continue;

            dtPolyRef fromRef = navMesh->encodePolyId(tile->salt, i, j);
            if (refToNode.find(fromRef) == refToNode.end()) continue;
            long fromNode = refToNode[fromRef];

            for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next) {
                dtPolyRef toRef = tile->links[k].ref;
                if (refToNode.find(toRef) == refToNode.end()) continue;
                long toNode = refToNode[toRef];
                if (fromNode == toNode) continue;

                float dist = distance(polyInfos[fromNode].center, polyInfos[toNode].center);
                unsigned char terrain = polyInfos[toNode].terrain;

                CostVector cost(0, costDim);
                cost[0] = static_cast<long>(dist * 100); // primary cost: distance * 100
                if (resourceTypes >= 1) {
                    cost[1] = static_cast<long>(getStaminaCost(terrain, dist) * 100);
                }
                if (resourceTypes >= 2) {
                    cost[2] = static_cast<long>(getAPCost(terrain, dist) * 100);
                }

                graph.AddEdge(fromNode, toNode, cost);
                edgeCount_++;
            }
        }
    }

    return graph;
}
#else
// Stub implementations when Detour is not available
void NavMeshConverter::computePolygonCenter(const dtMeshTile*, const dtPoly*, float*) {}

Roadmap NavMeshConverter::ConvertToRoadmap(const dtNavMesh*, int resourceTypes) {
    return Roadmap();
}
#endif

bool NavMeshConverter::ExportToDIMACS(const Roadmap& graph,
                                       const std::string& costFile,
                                       const std::vector<std::string>& resFiles) {
    long nNodes = graph.GetNumberOfNodes();
    long nEdges = graph.GetNumberOfEdges();

    // Export primary cost file
    {
        std::ofstream fout(costFile);
        if (!fout.is_open()) return false;
        fout << "c Primary cost (distance) file\n";
        fout << "p sp " << nNodes << " " << nEdges << "\n";

        for (auto& [u, neighbors] : graph.adjlist) {
            for (auto& [v, cost] : neighbors) {
                fout << "a " << u << " " << v << " " << cost[0] << "\n";
            }
        }
    }

    // Export resource files
    for (size_t r = 0; r < resFiles.size(); r++) {
        std::ofstream fout(resFiles[r]);
        if (!fout.is_open()) return false;
        fout << "c Resource " << (r + 1) << " file\n";
        fout << "p sp " << nNodes << " " << nEdges << "\n";

        for (auto& [u, neighbors] : graph.adjlist) {
            for (auto& [v, cost] : neighbors) {
                fout << "a " << u << " " << v << " " << cost[r + 1] << "\n";
            }
        }
    }

    return true;
}

} // namespace rcspp
