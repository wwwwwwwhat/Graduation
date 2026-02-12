#include <iostream>
#include <cstring>
#include <cmath>
#include <string>
#include <sstream>
#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "GraphConverter.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Generate a complex terrain mesh with height variation
void createComplexTerrain(
    int gridSize,       // grid dimension
    float cellSize,     // size per cell
    float*& vertices,
    int& vertCount,
    int*& triangles,
    int& triCount
) {
    vertCount = (gridSize + 1) * (gridSize + 1);
    triCount = gridSize * gridSize * 2;

    vertices = new float[vertCount * 3];
    triangles = new int[triCount * 3];

    int vertIndex = 0;
    for (int z = 0; z <= gridSize; ++z) {
        for (int x = 0; x <= gridSize; ++x) {
            float fx = (float)x / gridSize;
            float fz = (float)z / gridSize;

            // Height = gentle base + many pillars/walls as obstacles
            float height = 0.0f;
            // Gentle rolling base
            height += 0.3f * sinf(fx * 2.0f * (float)M_PI) * cosf(fz * 2.0f * (float)M_PI);

            // Dense grid of small pillars (non-walkable due to steep slope)
            // This forces the walkable area into many small connected regions
            float pillarSpacing = 3.0f / gridSize; // relative spacing
            float pillarFx = fmodf(fx, pillarSpacing) / pillarSpacing;
            float pillarFz = fmodf(fz, pillarSpacing) / pillarSpacing;
            // Pillar if close to grid intersection (small radius)
            float pdx = pillarFx - 0.5f;
            float pdz = pillarFz - 0.5f;
            float pillarDist = pdx * pdx + pdz * pdz;
            if (pillarDist < 0.03f) {
                height = 8.0f; // tall pillar, creates unwalkable steep slope
            }

            // Additional wall segments (horizontal and vertical)
            int gridX = (int)(fx * 8.0f);
            int gridZ = (int)(fz * 8.0f);
            // Horizontal walls with gaps
            if (gridZ % 2 == 0 && gridX % 3 != 0) {
                float wallFz = fmodf(fz * 8.0f, 1.0f);
                if (wallFz > 0.45f && wallFz < 0.55f) {
                    height = 6.0f;
                }
            }
            // Vertical walls with gaps
            if (gridX % 2 == 1 && gridZ % 3 != 1) {
                float wallFx = fmodf(fx * 8.0f, 1.0f);
                if (wallFx > 0.45f && wallFx < 0.55f) {
                    height = 6.0f;
                }
            }

            vertices[vertIndex * 3 + 0] = x * cellSize;
            vertices[vertIndex * 3 + 1] = height;
            vertices[vertIndex * 3 + 2] = z * cellSize;
            vertIndex++;
        }
    }

    // Generate triangles
    int triIndex = 0;
    for (int z = 0; z < gridSize; ++z) {
        for (int x = 0; x < gridSize; ++x) {
            int v0 = z * (gridSize + 1) + x;
            int v1 = v0 + 1;
            int v2 = (z + 1) * (gridSize + 1) + x;
            int v3 = v2 + 1;

            triangles[triIndex * 3 + 0] = v0;
            triangles[triIndex * 3 + 1] = v2;
            triangles[triIndex * 3 + 2] = v1;
            triIndex++;

            triangles[triIndex * 3 + 0] = v1;
            triangles[triIndex * 3 + 1] = v2;
            triangles[triIndex * 3 + 2] = v3;
            triIndex++;
        }
    }

    std::cout << "Terrain: " << vertCount << " vertices, "
              << triCount << " triangles, grid=" << gridSize << "x" << gridSize << std::endl;
}

// Build NavMesh with configurable parameters
dtNavMesh* buildNavMesh(
    const float* vertices, int vertCount,
    const int* triangles, int triCount,
    float cs,              // cell size (smaller = more polygons)
    float minRegArea,      // minimum region area
    float mergeRegArea     // merge region area
) {
    rcConfig config;
    memset(&config, 0, sizeof(config));

    float bmin[3] = {1e10f, 1e10f, 1e10f};
    float bmax[3] = {-1e10f, -1e10f, -1e10f};
    for (int i = 0; i < vertCount; ++i) {
        const float* v = &vertices[i * 3];
        for (int j = 0; j < 3; ++j) {
            bmin[j] = std::min(bmin[j], v[j]);
            bmax[j] = std::max(bmax[j], v[j]);
        }
    }

    config.cs = cs;
    config.ch = 0.2f;
    config.walkableSlopeAngle = 45.0f;
    config.walkableHeight = (int)ceilf(2.0f / config.ch);
    config.walkableClimb = (int)floorf(0.9f / config.ch);
    config.walkableRadius = (int)ceilf(0.3f / config.cs);
    config.maxEdgeLen = (int)(12.0f / config.cs);
    config.maxSimplificationError = 1.3f;
    config.minRegionArea = (int)rcSqr(minRegArea);
    config.mergeRegionArea = (int)rcSqr(mergeRegArea);
    config.maxVertsPerPoly = 6;
    config.detailSampleDist = 6.0f;
    config.detailSampleMaxError = 1.0f;

    rcVcopy(config.bmin, bmin);
    rcVcopy(config.bmax, bmax);
    rcCalcGridSize(config.bmin, config.bmax, config.cs, &config.width, &config.height);

    std::cout << "  Heightfield: " << config.width << "x" << config.height
              << ", cs=" << config.cs << ", minRegion=" << minRegArea << std::endl;

    rcContext ctx;
    rcHeightfield* hf = rcAllocHeightfield();
    if (!rcCreateHeightfield(&ctx, *hf, config.width, config.height,
                             config.bmin, config.bmax, config.cs, config.ch)) {
        std::cerr << "Failed to create heightfield" << std::endl;
        return nullptr;
    }

    unsigned char* triAreas = new unsigned char[triCount];
    memset(triAreas, 0, triCount * sizeof(unsigned char));
    rcMarkWalkableTriangles(&ctx, config.walkableSlopeAngle,
                           vertices, vertCount, triangles, triCount, triAreas);
    rcRasterizeTriangles(&ctx, vertices, vertCount, triangles, triAreas,
                        triCount, *hf, config.walkableClimb);
    delete[] triAreas;

    rcFilterLowHangingWalkableObstacles(&ctx, config.walkableClimb, *hf);
    rcFilterLedgeSpans(&ctx, config.walkableHeight, config.walkableClimb, *hf);
    rcFilterWalkableLowHeightSpans(&ctx, config.walkableHeight, *hf);

    rcCompactHeightfield* chf = rcAllocCompactHeightfield();
    if (!rcBuildCompactHeightfield(&ctx, config.walkableHeight, config.walkableClimb, *hf, *chf)) {
        std::cerr << "Failed to build compact heightfield" << std::endl;
        rcFreeHeightField(hf);
        return nullptr;
    }
    rcFreeHeightField(hf);

    if (!rcErodeWalkableArea(&ctx, config.walkableRadius, *chf)) {
        std::cerr << "Failed to erode" << std::endl;
        return nullptr;
    }

    if (!rcBuildDistanceField(&ctx, *chf)) {
        std::cerr << "Failed to build distance field" << std::endl;
        return nullptr;
    }

    if (!rcBuildRegions(&ctx, *chf, 0, config.minRegionArea, config.mergeRegionArea)) {
        std::cerr << "Failed to build regions" << std::endl;
        return nullptr;
    }

    rcContourSet* cset = rcAllocContourSet();
    if (!rcBuildContours(&ctx, *chf, config.maxSimplificationError, config.maxEdgeLen, *cset)) {
        std::cerr << "Failed to build contours" << std::endl;
        return nullptr;
    }

    rcPolyMesh* pmesh = rcAllocPolyMesh();
    if (!rcBuildPolyMesh(&ctx, *cset, config.maxVertsPerPoly, *pmesh)) {
        std::cerr << "Failed to build poly mesh" << std::endl;
        return nullptr;
    }

    rcPolyMeshDetail* dmesh = rcAllocPolyMeshDetail();
    if (!rcBuildPolyMeshDetail(&ctx, *pmesh, *chf,
                               config.detailSampleDist, config.detailSampleMaxError, *dmesh)) {
        std::cerr << "Failed to build detail mesh" << std::endl;
        return nullptr;
    }

    rcFreeCompactHeightfield(chf);
    rcFreeContourSet(cset);

    std::cout << "  PolyMesh: " << pmesh->npolys << " polygons" << std::endl;

    // Assign terrain types based on polygon position
    for (int i = 0; i < pmesh->npolys; ++i) {
        pmesh->flags[i] = 0x01;
        // Assign area based on position (hash of vertex positions)
        float cx = 0, cz = 0;
        const unsigned short* p = &pmesh->polys[i * pmesh->nvp * 2];
        int cnt = 0;
        for (int j = 0; j < pmesh->nvp; ++j) {
            if (p[j] == RC_MESH_NULL_IDX) break;
            cx += pmesh->verts[p[j] * 3 + 0];
            cz += pmesh->verts[p[j] * 3 + 2];
            cnt++;
        }
        if (cnt > 0) { cx /= cnt; cz /= cnt; }
        // Assign terrain: 0=ground, 1=grass, 2=road, 3=water, 4=mud
        int zone = ((int)(cx * 0.1f) + (int)(cz * 0.1f)) % 5;
        pmesh->areas[i] = (unsigned char)zone;
    }

    // Create Detour data
    dtNavMeshCreateParams params;
    memset(&params, 0, sizeof(params));
    params.verts = pmesh->verts;
    params.vertCount = pmesh->nverts;
    params.polys = pmesh->polys;
    params.polyAreas = pmesh->areas;
    params.polyFlags = pmesh->flags;
    params.polyCount = pmesh->npolys;
    params.nvp = pmesh->nvp;
    params.detailMeshes = dmesh->meshes;
    params.detailVerts = dmesh->verts;
    params.detailVertsCount = dmesh->nverts;
    params.detailTris = dmesh->tris;
    params.detailTriCount = dmesh->ntris;
    params.walkableHeight = 2.0f;
    params.walkableRadius = 0.3f;
    params.walkableClimb = 0.9f;
    rcVcopy(params.bmin, pmesh->bmin);
    rcVcopy(params.bmax, pmesh->bmax);
    params.cs = config.cs;
    params.ch = config.ch;
    params.buildBvTree = true;

    unsigned char* navData = nullptr;
    int navDataSize = 0;
    if (!dtCreateNavMeshData(&params, &navData, &navDataSize)) {
        std::cerr << "Failed to create Detour data" << std::endl;
        rcFreePolyMesh(pmesh);
        rcFreePolyMeshDetail(dmesh);
        return nullptr;
    }

    rcFreePolyMesh(pmesh);
    rcFreePolyMeshDetail(dmesh);

    dtNavMesh* navMesh = dtAllocNavMesh();
    dtStatus status = navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
    if (dtStatusFailed(status)) {
        dtFree(navData);
        dtFreeNavMesh(navMesh);
        std::cerr << "Failed to init navmesh" << std::endl;
        return nullptr;
    }

    return navMesh;
}

// Generate NavMesh and export DIMACS files
bool generateAndExport(int gridSize, float cellSize, float cs,
                       float minRegArea, float mergeRegArea,
                       const std::string& prefix) {
    float* vertices = nullptr;
    int vertCount = 0;
    int* triangles = nullptr;
    int triCount = 0;

    createComplexTerrain(gridSize, cellSize, vertices, vertCount, triangles, triCount);

    dtNavMesh* navMesh = buildNavMesh(vertices, vertCount, triangles, triCount,
                                      cs, minRegArea, mergeRegArea);
    delete[] vertices;
    delete[] triangles;

    if (!navMesh) {
        std::cerr << "NavMesh build failed for " << prefix << std::endl;
        return false;
    }

    Graph graph = GraphConverter::convertNavMeshToGraph(navMesh);
    dtFreeNavMesh(navMesh);

    if (graph.nodeCount() == 0) {
        std::cerr << "Empty graph for " << prefix << std::endl;
        return false;
    }

    std::string costFile = prefix + "_cost.gr";
    std::string res1File = prefix + "_res1.gr";
    std::string res2File = prefix + "_res2.gr";

    bool ok = GraphConverter::exportToDIMACSFormat(graph, costFile, res1File, res2File);
    if (ok) {
        std::cout << "  Exported: " << graph.nodeCount() << " nodes, "
                  << graph.edgeCount() << " edges -> " << prefix << "_*.gr" << std::endl;
    }
    return ok;
}

int main() {
    std::cout << "=== Large-Scale NavMesh Generator ===" << std::endl << std::endl;

    // Configuration: {gridSize, cellSize, cs, minRegArea, mergeRegArea, prefix}
    struct Config {
        int gridSize;
        float cellSize;
        float cs;
        float minRegArea;
        float mergeRegArea;
        std::string prefix;
    };

    Config configs[] = {
        // Small: target ~100+ polys
        {50,  1.0f, 0.3f, 2.0f, 4.0f,  "navmesh_small"},
        // Medium: target ~300+ polys
        {100, 1.0f, 0.2f, 1.5f, 3.0f,  "navmesh_medium"},
        // Large: target ~600+ polys
        {200, 1.0f, 0.15f, 1.0f, 2.0f, "navmesh_large"},
        // XLarge: target ~1000+ polys
        {300, 1.0f, 0.1f, 1.0f, 1.5f,  "navmesh_xlarge"},
    };

    for (auto& cfg : configs) {
        std::cout << "--- Generating " << cfg.prefix << " (grid="
                  << cfg.gridSize << ", cs=" << cfg.cs << ") ---" << std::endl;
        generateAndExport(cfg.gridSize, cfg.cellSize, cfg.cs,
                         cfg.minRegArea, cfg.mergeRegArea, cfg.prefix);
        std::cout << std::endl;
    }

    std::cout << "=== All NavMesh generated ===" << std::endl;
    return 0;
}
