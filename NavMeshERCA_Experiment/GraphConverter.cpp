#include "GraphConverter.h"
#include <cmath>
#include <fstream>
#include <iostream>

// 计算两点间距离
float GraphConverter::distance(const float* a, const float* b) {
    float dx = a[0] - b[0];
    float dy = a[1] - b[1];
    float dz = a[2] - b[2];
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

// 计算多边形中心点
void GraphConverter::computePolygonCenter(
    const dtMeshTile* tile,
    const dtPoly* poly,
    float* center
) {
    center[0] = 0.0f;
    center[1] = 0.0f;
    center[2] = 0.0f;

    for (int i = 0; i < poly->vertCount; ++i) {
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

// 根据地形类型获取体力消耗
float GraphConverter::getStaminaCost(unsigned char terrainType) {
    // 可以根据实际需求调整不同地形的体力消耗
    switch (terrainType) {
        case 0:  // 普通地面
            return 1.0f;
        case 1:  // 草地
            return 1.2f;
        case 2:  // 道路
            return 0.8f;
        case 3:  // 水域
            return 2.0f;
        case 4:  // 泥地
            return 1.5f;
        default:
            return 1.0f;
    }
}

// 根据地形类型获取行动点消耗
float GraphConverter::getAPCost(unsigned char terrainType) {
    // 可以根据实际需求调整不同地形的行动点消耗
    switch (terrainType) {
        case 0:  // 普通地面
            return 1.0f;
        case 1:  // 草地
            return 0.5f;
        case 2:  // 道路
            return 0.3f;
        case 3:  // 水域
            return 1.5f;
        case 4:  // 泥地
            return 1.0f;
        default:
            return 1.0f;
    }
}

// 将NavMesh转换为图结构
Graph GraphConverter::convertNavMeshToGraph(const dtNavMesh* navMesh) {
    Graph graph;

    if (!navMesh) {
        std::cerr << "错误: NavMesh为空" << std::endl;
        return graph;
    }

    // 获取所有tile
    int maxTiles = navMesh->getMaxTiles();

    // 第一步：为每个多边形创建节点
    for (int i = 0; i < maxTiles; ++i) {
        const dtMeshTile* tile = navMesh->getTile(i);
        if (!tile || !tile->header) continue;

        for (int j = 0; j < tile->header->polyCount; ++j) {
            const dtPoly* poly = &tile->polys[j];

            // 只处理地面多边形，跳过off-mesh连接
            if (poly->getType() != DT_POLYTYPE_GROUND)
                continue;

            GraphNode node;
            node.id = static_cast<int>(graph.nodes.size());
            computePolygonCenter(tile, poly, node.center);
            node.terrainType = poly->getArea();

            graph.nodes.push_back(node);
        }
    }

    std::cout << "创建了 " << graph.nodeCount() << " 个节点" << std::endl;

    // 第二步：为相邻多边形创建边
    int nodeIndex = 0;
    for (int i = 0; i < maxTiles; ++i) {
        const dtMeshTile* tile = navMesh->getTile(i);
        if (!tile || !tile->header) continue;

        for (int j = 0; j < tile->header->polyCount; ++j) {
            const dtPoly* poly = &tile->polys[j];

            if (poly->getType() != DT_POLYTYPE_GROUND) {
                continue;
            }

            // 遍历多边形的所有边
            for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next) {
                const dtLink& link = tile->links[k];

                // 获取邻接多边形
                if (link.ref == 0) continue;

                // 解码多边形引用以找到目标节点
                unsigned int salt, it, ip;
                navMesh->decodePolyId(link.ref, salt, it, ip);

                const dtMeshTile* targetTile = navMesh->getTile(it);
                if (!targetTile) continue;

                const dtPoly* targetPoly = &targetTile->polys[ip];
                if (targetPoly->getType() != DT_POLYTYPE_GROUND) continue;

                // 计算目标节点ID（简化版：需要遍历找到对应的节点）
                // 这里我们假设节点ID与多边形在tile中的顺序相关
                int targetNodeId = -1;
                int tempIndex = 0;
                for (int ti = 0; ti <= it; ++ti) {
                    const dtMeshTile* t = navMesh->getTile(ti);
                    if (!t || !t->header) continue;

                    int polyCount = (ti < it) ? t->header->polyCount : ip;
                    for (int pi = 0; pi < polyCount; ++pi) {
                        const dtPoly* p = &t->polys[pi];
                        if (p->getType() == DT_POLYTYPE_GROUND) {
                            tempIndex++;
                        }
                    }
                }
                targetNodeId = tempIndex;

                if (targetNodeId >= 0 && targetNodeId < graph.nodeCount()) {
                    GraphEdge edge;
                    edge.from = nodeIndex;
                    edge.to = targetNodeId;
                    edge.cost = distance(
                        graph.nodes[nodeIndex].center,
                        graph.nodes[targetNodeId].center
                    );
                    edge.resources[0] = getStaminaCost(graph.nodes[nodeIndex].terrainType) * edge.cost;
                    edge.resources[1] = getAPCost(graph.nodes[nodeIndex].terrainType);

                    graph.edges.push_back(edge);
                }
            }

            nodeIndex++;
        }
    }

    std::cout << "创建了 " << graph.edgeCount() << " 条边" << std::endl;

    return graph;
}

// 导出图为ERCA*输入格式
bool GraphConverter::exportToERCAFormat(
    const Graph& graph,
    int startNode,
    int endNode,
    float staminaLimit,
    float actionPointLimit,
    const std::string& filename
) {
    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }

    // 第一行：节点数 边数 资源维度数
    outFile << graph.nodeCount() << " "
            << graph.edgeCount() << " "
            << 2 << std::endl;

    outFile << std::endl;
    outFile << "# 边列表：起点 终点 成本 资源1(体力) 资源2(行动点)" << std::endl;

    // 边列表
    for (const auto& edge : graph.edges) {
        outFile << edge.from << " "
                << edge.to << " "
                << edge.cost << " "
                << edge.resources[0] << " "
                << edge.resources[1] << std::endl;
    }

    outFile << std::endl;
    outFile << "# 起点 终点 资源上限(体力 行动点)" << std::endl;

    // 查询参数：起点 终点 资源上限
    outFile << startNode << " "
            << endNode << " "
            << staminaLimit << " "
            << actionPointLimit << std::endl;

    outFile.close();

    std::cout << "成功导出ERCA*输入文件: " << filename << std::endl;
    std::cout << "  节点数: " << graph.nodeCount() << std::endl;
    std::cout << "  边数: " << graph.edgeCount() << std::endl;
    std::cout << "  起点: " << startNode << ", 终点: " << endNode << std::endl;
    std::cout << "  资源限制: 体力=" << staminaLimit
              << ", 行动点=" << actionPointLimit << std::endl;

    return true;
}
