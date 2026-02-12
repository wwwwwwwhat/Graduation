#include "graph/Roadmap.h"
#include "algorithm/ERCA.h"
#include "algorithm/Pulse.h"
#include "algorithm/Normalizer.h"
#include <iostream>
#include <vector>
#include <string>

using namespace rcspp;

// Build the 6-node NavMesh test graph manually
// This matches the graph from NavMeshERCA_Experiment
Roadmap buildTestGraph() {
    Roadmap graph;
    // costDim = 3: distance, stamina, action_points
    graph.Init(6, 3);

    // Add nodes (1-based, matching DIMACS)
    for (long i = 1; i <= 6; i++) {
        graph.AddNode(i);
    }

    // Edges from navmesh_cost.gr, navmesh_res1.gr, navmesh_res2.gr
    // Format: AddEdge(from, to, {distance, stamina, ap})
    graph.AddEdge(1, 5, CostVector({346, 195, 100}));
    graph.AddEdge(2, 4, CostVector({252, 189, 100}));
    graph.AddEdge(3, 5, CostVector({346, 294, 100}));
    graph.AddEdge(3, 6, CostVector({596, 453, 100}));
    graph.AddEdge(4, 2, CostVector({252, 189, 100}));
    graph.AddEdge(4, 6, CostVector({465, 352, 100}));
    graph.AddEdge(5, 3, CostVector({346, 294, 100}));
    graph.AddEdge(5, 1, CostVector({346, 195, 100}));
    graph.AddEdge(6, 3, CostVector({596, 453, 100}));
    graph.AddEdge(6, 4, CostVector({465, 352, 100}));

    return graph;
}

void testERCA() {
    std::cout << "=== Test ERCA* ===" << std::endl;

    Roadmap graph = buildTestGraph();
    std::cout << "Graph: " << graph.GetNumberOfNodes() << " nodes, "
              << graph.GetNumberOfEdges() << " edges, "
              << graph.GetCostDim() << " cost dimensions" << std::endl;

    long source = 1, target = 6;
    std::vector<long> resLimits = {5000, 3000}; // stamina, AP

    ERCA erca;
    erca.SetGraph(&graph);
    erca.SetResourceLimits(resLimits);
    erca.Search(source, target, 60.0);

    SearchResult res = erca.GetResult();
    std::cout << "Search time: " << res.rtSearch << "s" << std::endl;
    std::cout << "Init heuristic time: " << res.rtInitHeu << "s" << std::endl;
    std::cout << "Labels generated: " << res.nGenerated << std::endl;
    std::cout << "Labels expanded: " << res.nExpanded << std::endl;
    std::cout << "Timeout: " << (res.timeout ? "yes" : "no") << std::endl;

    if (res.paths.empty()) {
        std::cout << "No solution found!" << std::endl;
    } else {
        for (auto& [lid, path] : res.paths) {
            std::cout << "Path (label " << lid << "): ";
            for (size_t i = 0; i < path.size(); i++) {
                if (i > 0) std::cout << " -> ";
                std::cout << path[i];
            }
            std::cout << std::endl;
            std::cout << "Cost: " << res.costs[lid].ToStr() << std::endl;
        }
    }
    std::cout << std::endl;
}

void testPulse() {
    std::cout << "=== Test Pulse ===" << std::endl;

    Roadmap graph = buildTestGraph();
    long source = 1, target = 6;

    Pulse pulse;
    pulse.SetGraph(&graph);
    pulse.SetResourceLimit(5000, 1); // stamina limit on dim 1
    pulse.Search(source, target, 60.0);

    PulseResult res = pulse.GetResult();
    std::cout << "Feasible: " << (res.feasible ? "yes" : "no") << std::endl;
    std::cout << "Best cost: " << res.bestCost << std::endl;
    std::cout << "Search time: " << res.rtSearch << "s" << std::endl;
    std::cout << "Nodes explored: " << res.nExplored << std::endl;

    if (res.feasible) {
        std::cout << "Path: ";
        for (size_t i = 0; i < res.bestPath.size(); i++) {
            if (i > 0) std::cout << " -> ";
            std::cout << res.bestPath[i];
        }
        std::cout << std::endl;
        std::cout << "Full cost: " << res.bestCostVec.ToStr() << std::endl;
    }
    std::cout << std::endl;
}

void testNormalizerWithERCA() {
    std::cout << "=== Test Normalizer + ERCA* ===" << std::endl;

    Roadmap graph = buildTestGraph();
    long source = 1, target = 6;
    std::vector<long> resLimits = {5000, 3000};

    // Step 1: Normalization + Pulse upper bound
    Normalizer norm;
    norm.SetGraph(&graph);
    norm.SetResourceLimits(resLimits);
    NormResult normRes = norm.ComputeUpperBound(source, target, 5);

    std::cout << "Normalizer result:" << std::endl;
    std::cout << "  Feasible: " << (normRes.feasible ? "yes" : "no") << std::endl;
    std::cout << "  Iterations: " << normRes.iterations << std::endl;
    std::cout << "  Time: " << normRes.rtTotal << "s" << std::endl;
    if (normRes.feasible) {
        std::cout << "  Upper bound: " << normRes.upperBound.ToStr() << std::endl;
        std::cout << "  Path: ";
        for (size_t i = 0; i < normRes.path.size(); i++) {
            if (i > 0) std::cout << " -> ";
            std::cout << normRes.path[i];
        }
        std::cout << std::endl;
    }

    // Step 2: ERCA* with upper bound
    ERCA erca;
    erca.SetGraph(&graph);
    erca.SetResourceLimits(resLimits);
    if (normRes.feasible) {
        erca.SetUpperBound(normRes.upperBound);
    }
    erca.Search(source, target, 60.0);

    SearchResult res = erca.GetResult();
    std::cout << "\nERCA* with upper bound:" << std::endl;
    std::cout << "  Search time: " << res.rtSearch << "s" << std::endl;
    std::cout << "  Labels generated: " << res.nGenerated << std::endl;
    std::cout << "  Labels expanded: " << res.nExpanded << std::endl;

    if (!res.paths.empty()) {
        for (auto& [lid, path] : res.paths) {
            std::cout << "  Path: ";
            for (size_t i = 0; i < path.size(); i++) {
                if (i > 0) std::cout << " -> ";
                std::cout << path[i];
            }
            std::cout << std::endl;
            std::cout << "  Cost: " << res.costs[lid].ToStr() << std::endl;
        }
    }
    std::cout << std::endl;
}

void testLoadDIMACS() {
    std::cout << "=== Test DIMACS Loading ===" << std::endl;

    // Try to load the DIMACS files from previous experiment
    std::string basePath = "../NavMeshERCA_Experiment/build/Release/";
    std::string costFile = basePath + "navmesh_cost.gr";
    std::vector<std::string> resFiles = {
        basePath + "navmesh_res1.gr",
        basePath + "navmesh_res2.gr"
    };

    try {
        Roadmap graph = Roadmap::LoadDIMACS(costFile, resFiles);
        std::cout << "Loaded: " << graph.GetNumberOfNodes() << " nodes, "
                  << graph.GetNumberOfEdges() << " edges" << std::endl;

        ERCA erca;
        erca.SetGraph(&graph);
        erca.SetResourceLimits({5000, 3000});
        erca.Search(1, 6, 60.0);

        SearchResult res = erca.GetResult();
        if (!res.paths.empty()) {
            for (auto& [lid, path] : res.paths) {
                std::cout << "Path: ";
                for (size_t i = 0; i < path.size(); i++) {
                    if (i > 0) std::cout << " -> ";
                    std::cout << path[i];
                }
                std::cout << std::endl;
                std::cout << "Cost: " << res.costs[lid].ToStr() << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cout << "DIMACS loading skipped (files not found): " << e.what() << std::endl;
    }
    std::cout << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "ThesisRCSPP - Resource Constrained Shortest Path on NavMesh" << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cout << std::endl;

    testERCA();
    testPulse();
    testNormalizerWithERCA();
    testLoadDIMACS();

    std::cout << "All tests completed." << std::endl;
    return 0;
}
