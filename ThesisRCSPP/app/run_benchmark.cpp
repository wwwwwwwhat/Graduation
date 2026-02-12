#include "graph/Roadmap.h"
#include "benchmark/Benchmark.h"
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <random>

using namespace rcspp;

// Generate a random grid graph with multiple cost dimensions
// Simulates a NavMesh-like structure
Roadmap generateRandomGraph(int gridSize, int numResources, unsigned seed = 42) {
    std::mt19937 rng(seed);
    size_t costDim = 1 + numResources;

    Roadmap graph;
    int numNodes = gridSize * gridSize;
    graph.Init(numNodes, costDim);

    // Create grid nodes (1-based)
    for (int i = 1; i <= numNodes; i++) {
        graph.AddNode(i);
    }

    // Create grid edges (4-connected with some random diagonal connections)
    std::uniform_int_distribution<long> costDist(50, 500);
    std::uniform_int_distribution<long> resDist(20, 300);
    std::uniform_real_distribution<double> diagProb(0, 1);

    for (int y = 0; y < gridSize; y++) {
        for (int x = 0; x < gridSize; x++) {
            long nodeId = y * gridSize + x + 1;

            // Right neighbor
            if (x + 1 < gridSize) {
                long neighbor = y * gridSize + (x + 1) + 1;
                CostVector cost(0, costDim);
                cost[0] = costDist(rng);
                for (int r = 0; r < numResources; r++) {
                    cost[r + 1] = resDist(rng);
                }
                graph.AddEdge(nodeId, neighbor, cost);
                graph.AddEdge(neighbor, nodeId, cost);
            }
            // Down neighbor
            if (y + 1 < gridSize) {
                long neighbor = (y + 1) * gridSize + x + 1;
                CostVector cost(0, costDim);
                cost[0] = costDist(rng);
                for (int r = 0; r < numResources; r++) {
                    cost[r + 1] = resDist(rng);
                }
                graph.AddEdge(nodeId, neighbor, cost);
                graph.AddEdge(neighbor, nodeId, cost);
            }
            // Diagonal (30% chance)
            if (x + 1 < gridSize && y + 1 < gridSize && diagProb(rng) < 0.3) {
                long neighbor = (y + 1) * gridSize + (x + 1) + 1;
                CostVector cost(0, costDim);
                cost[0] = static_cast<long>(costDist(rng) * 1.414);
                for (int r = 0; r < numResources; r++) {
                    cost[r + 1] = static_cast<long>(resDist(rng) * 1.414);
                }
                graph.AddEdge(nodeId, neighbor, cost);
                graph.AddEdge(neighbor, nodeId, cost);
            }
        }
    }

    return graph;
}

void runBenchmarkSuite(const std::string& outputFile) {
    Benchmark bench;

    // Test configurations
    struct TestConfig {
        int gridSize;
        int numResources;
        std::string description;
    };

    std::vector<TestConfig> configs = {
        {10,  2, "100 nodes, 2 resources"},
        {22,  2, "~500 nodes, 2 resources"},
        {32,  2, "~1000 nodes, 2 resources"},
        {45,  2, "~2000 nodes, 2 resources"},
        {32,  1, "~1000 nodes, 1 resource"},
        {32,  3, "~1000 nodes, 3 resources"},
    };

    for (auto& cfg : configs) {
        std::cout << "=== Benchmark: " << cfg.description << " ===" << std::endl;

        Roadmap graph = generateRandomGraph(cfg.gridSize, cfg.numResources);
        long numNodes = graph.GetNumberOfNodes();
        long source = 1;
        long target = numNodes; // bottom-right corner

        // Resource limits: set to ~2x Dijkstra minimum resource cost
        // Average edge resource ~160, shortest path ~2*gridSize hops
        // Tight enough to enable pruning, loose enough for feasible solutions
        std::vector<long> resLimits;
        for (int r = 0; r < cfg.numResources; r++) {
            resLimits.push_back(cfg.gridSize * 250);
        }

        double timeLimit = 60.0;

        // 1. Dijkstra baseline
        std::cout << "  Running Dijkstra baseline..." << std::endl;
        auto dijkRes = bench.RunDijkstraBaseline(graph, source, target);
        std::cout << "    Time: " << dijkRes.rtSearch << "s, Cost: " << dijkRes.primaryCost << std::endl;

        // 2. Pure ERCA*
        std::cout << "  Running ERCA*..." << std::endl;
        auto ercaRes = bench.RunERCA(graph, source, target, resLimits, timeLimit);
        std::cout << "    Time: " << ercaRes.rtTotal << "s, Expanded: " << ercaRes.nExpanded;
        if (ercaRes.solved) std::cout << ", Cost: " << ercaRes.primaryCost;
        std::cout << std::endl;

        // 3. ERCA* with Pulse preprocessing
        std::cout << "  Running ERCA* + Pulse..." << std::endl;
        auto pulseRes = bench.RunERCAWithPulse(graph, source, target, resLimits, timeLimit);
        std::cout << "    Time: " << pulseRes.rtTotal << "s, Expanded: " << pulseRes.nExpanded;
        if (pulseRes.solved) std::cout << ", Cost: " << pulseRes.primaryCost;
        std::cout << std::endl;

        // Compare
        if (ercaRes.nExpanded > 0 && pulseRes.nExpanded > 0) {
            double reduction = 1.0 - static_cast<double>(pulseRes.nExpanded) / ercaRes.nExpanded;
            std::cout << "  Node reduction: " << (reduction * 100) << "%" << std::endl;
        }
        std::cout << std::endl;
    }

    // Export results
    bench.ExportCSV(outputFile);
    std::cout << "Results exported to: " << outputFile << std::endl;
}

int main(int argc, char** argv) {
    std::string outputFile = "benchmark_results.csv";
    if (argc > 1) {
        outputFile = argv[1];
    }

    std::cout << "ThesisRCSPP Benchmark Suite" << std::endl;
    std::cout << "===========================" << std::endl;
    std::cout << std::endl;

    runBenchmarkSuite(outputFile);

    return 0;
}
