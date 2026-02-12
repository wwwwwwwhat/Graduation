#ifndef GRAPH_CONVERTER_H
#define GRAPH_CONVERTER_H

#include <vector>
#include <string>
#include "DetourNavMesh.h"

// 图节点结构
struct GraphNode {
    int id;                    // 节点ID（对应多边形ID）
    float center[3];           // 多边形中心点坐标
    unsigned char terrainType; // 地形类型（区域类型）
};

// 图边结构
struct GraphEdge {
    int from;                  // 起始节点ID
    int to;                    // 目标节点ID
    float cost;                // 边的成本（距离）
    float resources[2];        // 资源消耗 [体力, 行动点]
};

// 图结构
struct Graph {
    std::vector<GraphNode> nodes;
    std::vector<GraphEdge> edges;

    // 统计信息
    int nodeCount() const { return static_cast<int>(nodes.size()); }
    int edgeCount() const { return static_cast<int>(edges.size()); }
};

// NavMesh到图的转换器
class GraphConverter {
public:
    // 将NavMesh转换为图结构
    static Graph convertNavMeshToGraph(const dtNavMesh* navMesh);

    // 导出图为ERCA*输入格式
    static bool exportToERCAFormat(
        const Graph& graph,
        int startNode,
        int endNode,
        float staminaLimit,
        float actionPointLimit,
        const std::string& filename
    );

    // Export graph to ERCA* DIMACS format (multiple files)
    static bool exportToDIMACSFormat(
        const Graph& graph,
        const std::string& costFile,
        const std::string& resource1File,
        const std::string& resource2File
    );

    // 计算多边形中心点
    static void computePolygonCenter(
        const dtMeshTile* tile,
        const dtPoly* poly,
        float* center
    );

    // 根据地形类型获取体力消耗
    static float getStaminaCost(unsigned char terrainType);

    // 根据地形类型获取行动点消耗
    static float getAPCost(unsigned char terrainType);

    // 计算两点间距离
    static float distance(const float* a, const float* b);
};

#endif // GRAPH_CONVERTER_H
