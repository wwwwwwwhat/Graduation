#include "GraphConverter.h"
#include <cmath>
#include <fstream>
#include <iostream>

// Calculate distance between two points
float GraphConverter::distance(const float* a, const float* b) {
    float dx = a[0] - b[0];
    float dy = a[1] - b[1];
    float dz = a[2] - b[2];
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

// Calculate polygon center
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

// Get stamina cost based on terrain type
float GraphConverter::getStaminaCost(unsigned char terrainType) {
    // Can be adjusted based on actual requirements
    switch (terrainType) {
        case 0:  // Normal ground
            return 1.0f;
        case 1:  // Grass
            return 1.2f;
        case 2:  // Road
            return 0.8f;
        case 3:  // Water
            return 2.0f;
        case 4:  // Mud
            return 1.5f;
        default:
            return 1.0f;
    }
}

// Get action point cost based on terrain type
float GraphConverter::getAPCost(unsigned char terrainType) {
    // Can be adjusted based on actual requirements
    switch (terrainType) {
        case 0:  // Normal ground
            return 1.0f;
        case 1:  // Grass
            return 0.5f;
        case 2:  // Road
            return 0.3f;
        case 3:  // Water
            return 1.5f;
        case 4:  // Mud
            return 1.0f;
        default:
            return 1.0f;
    }
}

// Convert NavMesh to graph structure
Graph GraphConverter::convertNavMeshToGraph(const dtNavMesh* navMesh) {
    Graph graph;

    if (!navMesh) {
        std::cerr << "Error: NavMesh is null" << std::endl;
        return graph;
    }

    // Get all tiles
    int maxTiles = navMesh->getMaxTiles();

    // Step 1: Create nodes for each polygon
    for (int i = 0; i < maxTiles; ++i) {
        const dtMeshTile* tile = navMesh->getTile(i);
        if (!tile || !tile->header) continue;

        for (int j = 0; j < tile->header->polyCount; ++j) {
            const dtPoly* poly = &tile->polys[j];

            // Only process ground polygons, skip off-mesh connections
            if (poly->getType() != DT_POLYTYPE_GROUND)
                continue;

            GraphNode node;
            node.id = static_cast<int>(graph.nodes.size());
            computePolygonCenter(tile, poly, node.center);
            node.terrainType = poly->getArea();

            graph.nodes.push_back(node);
        }
    }

    std::cout << "Created " << graph.nodeCount() << " nodes" << std::endl;

    // Step 2: Create edges for adjacent polygons
    int nodeIndex = 0;
    for (int i = 0; i < maxTiles; ++i) {
        const dtMeshTile* tile = navMesh->getTile(i);
        if (!tile || !tile->header) continue;

        for (int j = 0; j < tile->header->polyCount; ++j) {
            const dtPoly* poly = &tile->polys[j];

            if (poly->getType() != DT_POLYTYPE_GROUND) {
                continue;
            }

            // Traverse all edges of the polygon
            for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next) {
                const dtLink& link = tile->links[k];

                // Get adjacent polygon
                if (link.ref == 0) continue;

                // Decode polygon reference to find target node
                unsigned int salt, it, ip;
                navMesh->decodePolyId(link.ref, salt, it, ip);

                const dtMeshTile* targetTile = navMesh->getTile((int)it);
                if (!targetTile) continue;

                const dtPoly* targetPoly = &targetTile->polys[ip];
                if (targetPoly->getType() != DT_POLYTYPE_GROUND) continue;

                // Calculate target node ID (simplified: need to traverse to find corresponding node)
                // Here we assume node ID is related to polygon order in tile
                int targetNodeId = -1;
                int tempIndex = 0;
                for (int ti = 0; ti <= (int)it; ++ti) {
                    const dtMeshTile* t = navMesh->getTile(ti);
                    if (!t || !t->header) continue;

                    int polyCount = (ti < (int)it) ? t->header->polyCount : (int)ip;
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

    std::cout << "Created " << graph.edgeCount() << " edges" << std::endl;

    return graph;
}

// Export graph to ERCA* input format
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
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }

    // Line 1: node_count edge_count resource_dimensions
    outFile << graph.nodeCount() << " "
            << graph.edgeCount() << " "
            << 2 << std::endl;

    outFile << std::endl;
    outFile << "# Edge list: from to cost resource1(stamina) resource2(action_point)" << std::endl;

    // Edge list
    for (const auto& edge : graph.edges) {
        outFile << edge.from << " "
                << edge.to << " "
                << edge.cost << " "
                << edge.resources[0] << " "
                << edge.resources[1] << std::endl;
    }

    outFile << std::endl;
    outFile << "# Query: start end resource_limits(stamina action_point)" << std::endl;

    // Query parameters: start end resource_limits
    outFile << startNode << " "
            << endNode << " "
            << staminaLimit << " "
            << actionPointLimit << std::endl;

    outFile.close();

    std::cout << "Successfully exported ERCA* input file: " << filename << std::endl;
    std::cout << "  Nodes: " << graph.nodeCount() << std::endl;
    std::cout << "  Edges: " << graph.edgeCount() << std::endl;
    std::cout << "  Start: " << startNode << ", End: " << endNode << std::endl;
    std::cout << "  Resource limits: Stamina=" << staminaLimit
              << ", ActionPoint=" << actionPointLimit << std::endl;

    return true;
}

// Export graph to ERCA* DIMACS format (multiple files)
bool GraphConverter::exportToDIMACSFormat(
    const Graph& graph,
    const std::string& costFile,
    const std::string& resource1File,
    const std::string& resource2File
) {
    // Cost file (minimization objective)
    std::ofstream costOut(costFile);
    if (!costOut.is_open()) {
        std::cerr << "Failed to open cost file: " << costFile << std::endl;
        return false;
    }

    costOut << "c Cost file for NavMesh graph" << std::endl;
    costOut << "p sp " << graph.nodeCount() << " " << graph.edgeCount() << std::endl;

    for (const auto& edge : graph.edges) {
        // DIMACS format uses 1-based indexing
        costOut << "a " << (edge.from + 1) << " " << (edge.to + 1) << " "
                << static_cast<int>(edge.cost * 100) << std::endl;  // Scale and round
    }
    costOut.close();

    // Resource 1 file (stamina)
    std::ofstream res1Out(resource1File);
    if (!res1Out.is_open()) {
        std::cerr << "Failed to open resource1 file: " << resource1File << std::endl;
        return false;
    }

    res1Out << "c Resource 1 (Stamina) file for NavMesh graph" << std::endl;
    res1Out << "p sp " << graph.nodeCount() << " " << graph.edgeCount() << std::endl;

    for (const auto& edge : graph.edges) {
        res1Out << "a " << (edge.from + 1) << " " << (edge.to + 1) << " "
                << static_cast<int>(edge.resources[0] * 100) << std::endl;
    }
    res1Out.close();

    // Resource 2 file (action points)
    std::ofstream res2Out(resource2File);
    if (!res2Out.is_open()) {
        std::cerr << "Failed to open resource2 file: " << resource2File << std::endl;
        return false;
    }

    res2Out << "c Resource 2 (Action Points) file for NavMesh graph" << std::endl;
    res2Out << "p sp " << graph.nodeCount() << " " << graph.edgeCount() << std::endl;

    for (const auto& edge : graph.edges) {
        res2Out << "a " << (edge.from + 1) << " " << (edge.to + 1) << " "
                << static_cast<int>(edge.resources[1] * 100) << std::endl;
    }
    res2Out.close();

    std::cout << "Successfully exported DIMACS format files:" << std::endl;
    std::cout << "  Cost file: " << costFile << std::endl;
    std::cout << "  Resource 1 file: " << resource1File << std::endl;
    std::cout << "  Resource 2 file: " << resource2File << std::endl;

    return true;
}
