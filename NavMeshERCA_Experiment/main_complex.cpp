#include <iostream>
#include <cstring>
#include <vector>
#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "GraphConverter.h"

// Create a more complex mesh with obstacles
void createComplexMesh(
    float*& vertices,
    int& vertCount,
    int*& triangles,
    int& triCount
) {
    std::vector<float> verts;
    std::vector<int> tris;

    // Create a terrain with multiple platforms and gaps
    // Platform 1: Bottom-left
    float p1_x = 0.0f, p1_z = 0.0f;
    float p1_size = 5.0f;
    addPlatform(verts, tris, p1_x, p1_z, p1_size, p1_size, 0.0f);

    // Platform 2: Middle (slightly elevated)
    float p2_x = 6.0f, p2_z = 2.0f;
    float p2_size_x = 4.0f, p2_size_z = 6.0f;
    addPlatform(verts, tris, p2_x, p2_z, p2_size_x, p2_size_z, 0.5f);

    // Platform 3: Top-right
    float p3_x = 11.0f, p3_z = 4.0f;
    float p3_size = 5.0f;
    addPlatform(verts, tris, p3_x, p3_z, p3_size, p3_size, 0.0f);

    // Platform 4: Bottom-right
    float p4_x = 8.0f, p4_z = 10.0f;
    float p4_size_x = 6.0f, p4_size_z = 4.0f;
    addPlatform(verts, tris, p4_x, p4_z, p4_size_x, p4_size_z, 0.2f);

    vertCount = (int)(verts.size() / 3);
    triCount = (int)(tris.size() / 3);

    vertices = new float[verts.size()];
    triangles = new int[tris.size()];

    memcpy(vertices, &verts[0], verts.size() * sizeof(float));
    memcpy(triangles, &tris[0], tris.size() * sizeof(int));

    std::cout << "Created complex mesh: " << vertCount << " vertices, "
              << triCount << " triangles" << std::endl;
}

void addPlatform(
    std::vector<float>& verts,
    std::vector<int>& tris,
    float x, float z,
    float width, float depth,
    float height
) {
    int baseIndex = (int)(verts.size() / 3);

    // Add 4 vertices for the platform
    verts.push_back(x);
    verts.push_back(height);
    verts.push_back(z);

    verts.push_back(x + width);
    verts.push_back(height);
    verts.push_back(z);

    verts.push_back(x + width);
    verts.push_back(height);
    verts.push_back(z + depth);

    verts.push_back(x);
    verts.push_back(height);
    verts.push_back(z + depth);

    // Add 2 triangles for the platform
    tris.push_back(baseIndex + 0);
    tris.push_back(baseIndex + 1);
    tris.push_back(baseIndex + 2);

    tris.push_back(baseIndex + 0);
    tris.push_back(baseIndex + 2);
    tris.push_back(baseIndex + 3);
}

// Build NavMesh (same as before)
dtNavMesh* buildNavMesh(
    const float* vertices,
    int vertCount,
    const int* triangles,
    int triCount
) {
    // Config parameters
    rcConfig config;
    memset(&config, 0, sizeof(config));

    // Calculate bounds
    float bmin[3] = {1e10f, 1e10f, 1e10f};
    float bmax[3] = {-1e10f, -1e10f, -1e10f};
    for (int i = 0; i < vertCount; ++i) {
        const float* v = &vertices[i * 3];
        for (int j = 0; j < 3; ++j) {
            bmin[j] = std::min(bmin[j], v[j]);
            bmax[j] = std::max(bmax[j], v[j]);
        }
    }

    // Setup configuration
    config.cs = 0.3f;           // cell size
    config.ch = 0.2f;           // cell height
    config.walkableSlopeAngle = 45.0f;
    config.walkableHeight = (int)ceilf(2.0f / config.ch);
    config.walkableClimb = (int)floorf(0.9f / config.ch);
    config.walkableRadius = (int)ceilf(0.6f / config.cs);
    config.maxEdgeLen = (int)(12.0f / config.cs);
    config.maxSimplificationError = 1.3f;
    config.minRegionArea = (int)rcSqr(8);
    config.mergeRegionArea = (int)rcSqr(20);
    config.maxVertsPerPoly = 6;
    config.detailSampleDist = 6.0f;
    config.detailSampleMaxError = 1.0f;

    rcVcopy(config.bmin, bmin);
    rcVcopy(config.bmax, bmax);
    rcCalcGridSize(config.bmin, config.bmax, config.cs, &config.width, &config.height);

    std::cout << "NavMesh configuration:" << std::endl;
    std::cout << "  Grid size: " << config.width << " x " << config.height << std::endl;
    std::cout << "  Cell size: " << config.cs << std::endl;

    // Allocate Recast structures
    rcContext ctx;
    rcHeightfield* heightfield = rcAllocHeightfield();
    if (!rcCreateHeightfield(&ctx, *heightfield, config.width, config.height,
                             config.bmin, config.bmax, config.cs, config.ch)) {
        std::cerr << "Failed to create heightfield" << std::endl;
        return nullptr;
    }

    // Rasterize triangles
    unsigned char* triAreas = new unsigned char[triCount];
    memset(triAreas, 0, triCount * sizeof(unsigned char));
    rcMarkWalkableTriangles(&ctx, config.walkableSlopeAngle,
                           vertices, vertCount, triangles, triCount, triAreas);
    rcRasterizeTriangles(&ctx, vertices, vertCount, triangles, triAreas,
                        triCount, *heightfield, config.walkableClimb);
    delete[] triAreas;

    // Filter
    rcFilterLowHangingWalkableObstacles(&ctx, config.walkableClimb, *heightfield);
    rcFilterLedgeSpans(&ctx, config.walkableHeight, config.walkableClimb, *heightfield);
    rcFilterWalkableLowHeightSpans(&ctx, config.walkableHeight, *heightfield);

    // Compact heightfield
    rcCompactHeightfield* compactHeightfield = rcAllocCompactHeightfield();
    if (!rcBuildCompactHeightfield(&ctx, config.walkableHeight, config.walkableClimb,
                                   *heightfield, *compactHeightfield)) {
        std::cerr << "Failed to build compact heightfield" << std::endl;
        return nullptr;
    }
    rcFreeHeightField(heightfield);

    // Erode walkable area
    if (!rcErodeWalkableArea(&ctx, config.walkableRadius, *compactHeightfield)) {
        std::cerr << "Failed to erode walkable area" << std::endl;
        return nullptr;
    }

    // Build distance field
    if (!rcBuildDistanceField(&ctx, *compactHeightfield)) {
        std::cerr << "Failed to build distance field" << std::endl;
        return nullptr;
    }

    // Build regions
    if (!rcBuildRegions(&ctx, *compactHeightfield, 0, config.minRegionArea,
                       config.mergeRegionArea)) {
        std::cerr << "Failed to build regions" << std::endl;
        return nullptr;
    }

    // Build contours
    rcContourSet* contourSet = rcAllocContourSet();
    if (!rcBuildContours(&ctx, *compactHeightfield, config.maxSimplificationError,
                        config.maxEdgeLen, *contourSet)) {
        std::cerr << "Failed to build contours" << std::endl;
        return nullptr;
    }

    // Build polygon mesh
    rcPolyMesh* polyMesh = rcAllocPolyMesh();
    if (!rcBuildPolyMesh(&ctx, *contourSet, config.maxVertsPerPoly, *polyMesh)) {
        std::cerr << "Failed to build polygon mesh" << std::endl;
        return nullptr;
    }

    // Build detail mesh
    rcPolyMeshDetail* detailMesh = rcAllocPolyMeshDetail();
    if (!rcBuildPolyMeshDetail(&ctx, *polyMesh, *compactHeightfield,
                               config.detailSampleDist, config.detailSampleMaxError,
                               *detailMesh)) {
        std::cerr << "Failed to build detail mesh" << std::endl;
        return nullptr;
    }

    // Cleanup
    rcFreeCompactHeightfield(compactHeightfield);
    rcFreeContourSet(contourSet);

    std::cout << "Polygon mesh: " << polyMesh->npolys << " polygons" << std::endl;

    // Create Detour navmesh data
    for (int i = 0; i < polyMesh->npolys; ++i) {
        polyMesh->flags[i] = 0x01;  // walkable flag
        polyMesh->areas[i] = 0;     // default area
    }

    dtNavMeshCreateParams params;
    memset(&params, 0, sizeof(params));
    params.verts = polyMesh->verts;
    params.vertCount = polyMesh->nverts;
    params.polys = polyMesh->polys;
    params.polyAreas = polyMesh->areas;
    params.polyFlags = polyMesh->flags;
    params.polyCount = polyMesh->npolys;
    params.nvp = polyMesh->nvp;
    params.detailMeshes = detailMesh->meshes;
    params.detailVerts = detailMesh->verts;
    params.detailVertsCount = detailMesh->nverts;
    params.detailTris = detailMesh->tris;
    params.detailTriCount = detailMesh->ntris;
    params.walkableHeight = 2.0f;
    params.walkableRadius = 0.6f;
    params.walkableClimb = 0.9f;
    rcVcopy(params.bmin, polyMesh->bmin);
    rcVcopy(params.bmax, polyMesh->bmax);
    params.cs = config.cs;
    params.ch = config.ch;
    params.buildBvTree = true;

    unsigned char* navData = nullptr;
    int navDataSize = 0;
    if (!dtCreateNavMeshData(&params, &navData, &navDataSize)) {
        std::cerr << "Failed to create Detour navmesh data" << std::endl;
        return nullptr;
    }

    // Cleanup
    rcFreePolyMesh(polyMesh);
    rcFreePolyMeshDetail(detailMesh);

    // Create navmesh
    dtNavMesh* navMesh = dtAllocNavMesh();
    if (!navMesh) {
        dtFree(navData);
        std::cerr << "Failed to allocate navmesh" << std::endl;
        return nullptr;
    }

    dtStatus status = navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
    if (dtStatusFailed(status)) {
        dtFree(navData);
        dtFreeNavMesh(navMesh);
        std::cerr << "Failed to initialize navmesh" << std::endl;
        return nullptr;
    }

    std::cout << "NavMesh built successfully!" << std::endl;
    return navMesh;
}

int main() {
    std::cout << "=== NavMesh to ERCA* Minimal Viable Experiment ===" << std::endl;
    std::cout << std::endl;

    // Step 1: Create complex test mesh
    std::cout << "[Step 1] Creating complex test mesh..." << std::endl;
    float* vertices = nullptr;
    int vertCount = 0;
    int* triangles = nullptr;
    int triCount = 0;
    createComplexMesh(vertices, vertCount, triangles, triCount);
    std::cout << std::endl;

    // Step 2: Build NavMesh
    std::cout << "[Step 2] Building NavMesh..." << std::endl;
    dtNavMesh* navMesh = buildNavMesh(vertices, vertCount, triangles, triCount);
    if (!navMesh) {
        std::cerr << "NavMesh build failed!" << std::endl;
        delete[] vertices;
        delete[] triangles;
        return 1;
    }
    std::cout << std::endl;

    // Step 3: Convert to graph structure
    std::cout << "[Step 3] Converting NavMesh to graph..." << std::endl;
    Graph graph = GraphConverter::convertNavMeshToGraph(navMesh);
    std::cout << std::endl;

    // Step 4: Export ERCA* input format
    std::cout << "[Step 4] Exporting ERCA* input file..." << std::endl;
    if (graph.nodeCount() > 0) {
        int startNode = 0;
        int endNode = graph.nodeCount() - 1;
        float staminaLimit = 50.0f;
        float actionPointLimit = 30.0f;

        bool success = GraphConverter::exportToERCAFormat(
            graph,
            startNode,
            endNode,
            staminaLimit,
            actionPointLimit,
            "erca_input.txt"
        );

        if (!success) {
            std::cerr << "Failed to export ERCA* input file!" << std::endl;
        }
    } else {
        std::cerr << "Graph is empty, cannot export!" << std::endl;
    }
    std::cout << std::endl;

    // Cleanup
    dtFreeNavMesh(navMesh);
    delete[] vertices;
    delete[] triangles;

    std::cout << "=== Experiment Complete ===" << std::endl;
    std::cout << "ERCA* input file generated: erca_input.txt" << std::endl;
    std::cout << "Next step: Use ERCA* solver to process this file" << std::endl;

    return 0;
}
