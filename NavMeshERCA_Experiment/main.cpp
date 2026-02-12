#include <iostream>
#include <cstring>
#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "GraphConverter.h"

// Create a simple ground mesh for testing
void createSimpleGroundMesh(
    float*& vertices,
    int& vertCount,
    int*& triangles,
    int& triCount
) {
    // Create a 10x10 grid plane
    const int gridSize = 10;
    const float cellSize = 1.0f;

    vertCount = (gridSize + 1) * (gridSize + 1);
    triCount = gridSize * gridSize * 2;

    vertices = new float[vertCount * 3];
    triangles = new int[triCount * 3];

    // Generate vertices
    int vertIndex = 0;
    for (int z = 0; z <= gridSize; ++z) {
        for (int x = 0; x <= gridSize; ++x) {
            vertices[vertIndex * 3 + 0] = x * cellSize;
            vertices[vertIndex * 3 + 1] = 0.0f; // plane height
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

            // First triangle
            triangles[triIndex * 3 + 0] = v0;
            triangles[triIndex * 3 + 1] = v2;
            triangles[triIndex * 3 + 2] = v1;
            triIndex++;

            // Second triangle
            triangles[triIndex * 3 + 0] = v1;
            triangles[triIndex * 3 + 1] = v2;
            triangles[triIndex * 3 + 2] = v3;
            triIndex++;
        }
    }

    std::cout << "Created simple mesh: " << vertCount << " vertices, "
              << triCount << " triangles" << std::endl;
}

// Build NavMesh
dtNavMesh* buildNavMesh(
    const float* vertices,
    int vertCount,
    const int* triangles,
    int triCount
) {
    // 配置参数
    rcConfig config;
    memset(&config, 0, sizeof(config));

    // 计算边界
    float bmin[3] = {1e10f, 1e10f, 1e10f};
    float bmax[3] = {-1e10f, -1e10f, -1e10f};
    for (int i = 0; i < vertCount; ++i) {
        const float* v = &vertices[i * 3];
        for (int j = 0; j < 3; ++j) {
            bmin[j] = std::min(bmin[j], v[j]);
            bmax[j] = std::max(bmax[j], v[j]);
        }
    }

    // 设置配置
    config.cs = 0.3f;           // 单元格大小
    config.ch = 0.2f;           // 单元格高度
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

    std::cout << "NavMesh配置:" << std::endl;
    std::cout << "  网格大小: " << config.width << " x " << config.height << std::endl;
    std::cout << "  单元格大小: " << config.cs << std::endl;

    // 分配Recast数据结构
    rcContext ctx;
    rcHeightfield* heightfield = rcAllocHeightfield();
    if (!rcCreateHeightfield(&ctx, *heightfield, config.width, config.height,
                             config.bmin, config.bmax, config.cs, config.ch)) {
        std::cerr << "无法创建高度场" << std::endl;
        return nullptr;
    }

    // 光栅化三角形
    unsigned char* triAreas = new unsigned char[triCount];
    memset(triAreas, 0, triCount * sizeof(unsigned char));
    rcMarkWalkableTriangles(&ctx, config.walkableSlopeAngle,
                           vertices, vertCount, triangles, triCount, triAreas);
    rcRasterizeTriangles(&ctx, vertices, vertCount, triangles, triAreas,
                        triCount, *heightfield, config.walkableClimb);
    delete[] triAreas;

    // 过滤
    rcFilterLowHangingWalkableObstacles(&ctx, config.walkableClimb, *heightfield);
    rcFilterLedgeSpans(&ctx, config.walkableHeight, config.walkableClimb, *heightfield);
    rcFilterWalkableLowHeightSpans(&ctx, config.walkableHeight, *heightfield);

    // 压缩高度场
    rcCompactHeightfield* compactHeightfield = rcAllocCompactHeightfield();
    if (!rcBuildCompactHeightfield(&ctx, config.walkableHeight, config.walkableClimb,
                                   *heightfield, *compactHeightfield)) {
        std::cerr << "无法构建压缩高度场" << std::endl;
        return nullptr;
    }
    rcFreeHeightField(heightfield);

    // 侵蚀可行走区域
    if (!rcErodeWalkableArea(&ctx, config.walkableRadius, *compactHeightfield)) {
        std::cerr << "无法侵蚀可行走区域" << std::endl;
        return nullptr;
    }

    // 构建距离场
    if (!rcBuildDistanceField(&ctx, *compactHeightfield)) {
        std::cerr << "无法构建距离场" << std::endl;
        return nullptr;
    }

    // 构建区域
    if (!rcBuildRegions(&ctx, *compactHeightfield, 0, config.minRegionArea,
                       config.mergeRegionArea)) {
        std::cerr << "无法构建区域" << std::endl;
        return nullptr;
    }

    // 构建轮廓
    rcContourSet* contourSet = rcAllocContourSet();
    if (!rcBuildContours(&ctx, *compactHeightfield, config.maxSimplificationError,
                        config.maxEdgeLen, *contourSet)) {
        std::cerr << "无法构建轮廓" << std::endl;
        return nullptr;
    }

    // 构建多边形网格
    rcPolyMesh* polyMesh = rcAllocPolyMesh();
    if (!rcBuildPolyMesh(&ctx, *contourSet, config.maxVertsPerPoly, *polyMesh)) {
        std::cerr << "无法构建多边形网格" << std::endl;
        return nullptr;
    }

    // 构建细节网格
    rcPolyMeshDetail* detailMesh = rcAllocPolyMeshDetail();
    if (!rcBuildPolyMeshDetail(&ctx, *polyMesh, *compactHeightfield,
                               config.detailSampleDist, config.detailSampleMaxError,
                               *detailMesh)) {
        std::cerr << "无法构建细节网格" << std::endl;
        return nullptr;
    }

    // 清理
    rcFreeCompactHeightfield(compactHeightfield);
    rcFreeContourSet(contourSet);

    std::cout << "多边形网格: " << polyMesh->npolys << " 个多边形" << std::endl;

    // 创建Detour navmesh数据
    for (int i = 0; i < polyMesh->npolys; ++i) {
        polyMesh->flags[i] = 0x01;  // 可行走标志
        polyMesh->areas[i] = 0;     // 默认区域
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
        std::cerr << "无法创建Detour navmesh数据" << std::endl;
        return nullptr;
    }

    // 清理
    rcFreePolyMesh(polyMesh);
    rcFreePolyMeshDetail(detailMesh);

    // 创建navmesh
    dtNavMesh* navMesh = dtAllocNavMesh();
    if (!navMesh) {
        dtFree(navData);
        std::cerr << "无法分配navmesh" << std::endl;
        return nullptr;
    }

    dtStatus status = navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
    if (dtStatusFailed(status)) {
        dtFree(navData);
        dtFreeNavMesh(navMesh);
        std::cerr << "无法初始化navmesh" << std::endl;
        return nullptr;
    }

    std::cout << "NavMesh构建成功!" << std::endl;
    return navMesh;
}

int main() {
    std::cout << "=== NavMesh到ERCA*最小可行实验 ===" << std::endl;
    std::cout << std::endl;

    // 步骤1: 创建简单的测试网格
    std::cout << "[步骤1] 创建测试网格..." << std::endl;
    float* vertices = nullptr;
    int vertCount = 0;
    int* triangles = nullptr;
    int triCount = 0;
    createSimpleGroundMesh(vertices, vertCount, triangles, triCount);
    std::cout << std::endl;

    // 步骤2: 构建NavMesh
    std::cout << "[步骤2] 构建NavMesh..." << std::endl;
    dtNavMesh* navMesh = buildNavMesh(vertices, vertCount, triangles, triCount);
    if (!navMesh) {
        std::cerr << "NavMesh构建失败!" << std::endl;
        delete[] vertices;
        delete[] triangles;
        return 1;
    }
    std::cout << std::endl;

    // 步骤3: 转换为图结构
    std::cout << "[步骤3] 转换NavMesh为图结构..." << std::endl;
    Graph graph = GraphConverter::convertNavMeshToGraph(navMesh);
    std::cout << std::endl;

    // 步骤4: 导出ERCA*输入格式
    std::cout << "[步骤4] 导出ERCA*输入文件..." << std::endl;
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
            std::cerr << "导出ERCA*输入文件失败!" << std::endl;
        }
    } else {
        std::cerr << "图为空，无法导出!" << std::endl;
    }
    std::cout << std::endl;

    // 清理
    dtFreeNavMesh(navMesh);
    delete[] vertices;
    delete[] triangles;

    std::cout << "=== 实验完成 ===" << std::endl;
    std::cout << "ERCA*输入文件已生成: erca_input.txt" << std::endl;
    std::cout << "下一步: 使用ERCA*求解器处理此文件" << std::endl;

    return 0;
}
