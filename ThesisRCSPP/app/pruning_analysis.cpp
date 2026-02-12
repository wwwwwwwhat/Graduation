#include "graph/Roadmap.h"
#include "algorithm/ERCA.h"
#include "algorithm/Pulse.h"
#include "algorithm/Normalizer.h"
#include "algorithm/Dijkstra.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <random>
#include <chrono>
#include <iomanip>
#include <cmath>

using namespace rcspp;

Roadmap generateGraph(int gridSize, int numResources, double extraEdgeFraction, unsigned seed = 42) {
    std::mt19937 rng(seed);
    size_t costDim = 1 + numResources;

    Roadmap graph;
    int numNodes = gridSize * gridSize;
    graph.Init(numNodes, costDim);

    for (int i = 1; i <= numNodes; i++) {
        graph.AddNode(i);
    }

    std::uniform_int_distribution<long> costDist(50, 500);
    std::uniform_int_distribution<long> resDist(20, 300);
    std::uniform_real_distribution<double> edgeProb(0, 1);

    auto addEdgePair = [&](long from, long to) {
        CostVector cost(0, costDim);
        cost[0] = costDist(rng);
        for (int r = 0; r < numResources; r++) {
            cost[r + 1] = resDist(rng);
        }
        graph.AddEdge(from, to, cost);
        graph.AddEdge(to, from, cost);
    };

    for (int y = 0; y < gridSize; y++) {
        for (int x = 0; x < gridSize; x++) {
            long nodeId = y * gridSize + x + 1;
            if (x + 1 < gridSize) addEdgePair(nodeId, nodeId + 1);
            if (y + 1 < gridSize) addEdgePair(nodeId, nodeId + gridSize);
        }
    }

    for (int y = 0; y < gridSize; y++) {
        for (int x = 0; x < gridSize; x++) {
            long nodeId = y * gridSize + x + 1;
            if (x + 1 < gridSize && y + 1 < gridSize && edgeProb(rng) < extraEdgeFraction) {
                addEdgePair(nodeId, (y + 1) * gridSize + (x + 1) + 1);
            }
            if (x > 0 && y + 1 < gridSize && edgeProb(rng) < extraEdgeFraction * 0.5) {
                addEdgePair(nodeId, (y + 1) * gridSize + (x - 1) + 1);
            }
        }
    }

    return graph;
}

struct MethodResult {
    std::string method;
    double totalTimeMs;
    double normTimeMs;   // preprocessing time (0 for pure ERCA*)
    double searchTimeMs;
    long expanded;
    long generated;
    long prunedResource;
    long prunedUpperBound;
    long prunedDominance;
    long cost;
    bool solved;
    bool normFeasible;
    long ubCost;
};

struct TestRow {
    std::string config;
    int gridSize;
    int numRes;
    double tightness;
    long nNodes;
    long nEdges;
    MethodResult baseline;     // ERCA* (w=1.0, no UB)
    MethodResult withUB;       // ERCA* (w=1.0, with Norm+ERCA UB)
    MethodResult inflated;     // ERCA* (w=1.5, no UB) -- weighted A* baseline
    MethodResult inflatedUB;   // ERCA* (w=1.5, with Norm+ERCA UB) -- full strategy
};

MethodResult runERCA(Roadmap& graph, long source, long target,
                     const std::vector<long>& resLimits, double w, double timeLimit) {
    MethodResult r;
    r.method = (w > 1.01) ? "ERCA*(w=" + std::to_string(w).substr(0, 3) + ")" : "ERCA*";
    r.normTimeMs = 0;
    r.normFeasible = false;
    r.ubCost = -1;

    ERCA erca;
    erca.SetGraph(&graph);
    erca.SetResourceLimits(resLimits);
    erca.SetHeuInflateRate(w);

    auto t0 = std::chrono::steady_clock::now();
    erca.Search(source, target, timeLimit);
    auto t1 = std::chrono::steady_clock::now();

    SearchResult res = erca.GetResult();
    r.totalTimeMs = std::chrono::duration<double>(t1 - t0).count() * 1000;
    r.searchTimeMs = r.totalTimeMs;
    r.expanded = res.nExpanded;
    r.generated = res.nGenerated;
    r.prunedResource = res.nPrunedByResource;
    r.prunedUpperBound = res.nPrunedByUpperBound;
    r.prunedDominance = res.nPrunedByDominance;
    r.solved = !res.paths.empty();
    r.cost = r.solved ? res.costs.begin()->second[0] : -1;
    return r;
}

MethodResult runERCAWithNormUB(Roadmap& graph, long source, long target,
                               const std::vector<long>& resLimits, double w, double timeLimit) {
    MethodResult r;
    r.method = (w > 1.01) ? "Norm+ERCA*(w=" + std::to_string(w).substr(0,3) + ")" : "Norm+ERCA*";

    auto t0 = std::chrono::steady_clock::now();

    // Phase 1: Normalization + ERCA* on normalized graph
    Normalizer norm;
    norm.SetGraph(&graph);
    norm.SetResourceLimits(resLimits);
    NormResult normRes = norm.ComputeUpperBoundERCA(source, target, 5);

    auto tNorm = std::chrono::steady_clock::now();
    r.normTimeMs = std::chrono::duration<double>(tNorm - t0).count() * 1000;
    r.normFeasible = normRes.feasible;
    r.ubCost = normRes.feasible ? normRes.upperBound[0] : -1;

    // Phase 2: ERCA* with upper bound + optional inflation
    ERCA erca;
    erca.SetGraph(&graph);
    erca.SetResourceLimits(resLimits);
    erca.SetHeuInflateRate(w);
    if (normRes.feasible) {
        erca.SetUpperBound(normRes.upperBound);
    }
    erca.Search(source, target, timeLimit);

    auto t1 = std::chrono::steady_clock::now();
    SearchResult res = erca.GetResult();
    r.totalTimeMs = std::chrono::duration<double>(t1 - t0).count() * 1000;
    r.searchTimeMs = r.totalTimeMs - r.normTimeMs;
    r.expanded = res.nExpanded;
    r.generated = res.nGenerated;
    r.prunedResource = res.nPrunedByResource;
    r.prunedUpperBound = res.nPrunedByUpperBound;
    r.prunedDominance = res.nPrunedByDominance;
    r.solved = !res.paths.empty();
    r.cost = r.solved ? res.costs.begin()->second[0] : -1;
    return r;
}

int main() {
    std::cout << "=== Pruning Strategy Analysis (Improved) ===" << std::endl;
    std::cout << "Methods:" << std::endl;
    std::cout << "  A: ERCA* (w=1.0, no UB)       -- exact baseline" << std::endl;
    std::cout << "  B: Norm+ERCA* (w=1.0, with UB) -- normalization + UB pruning" << std::endl;
    std::cout << "  C: ERCA* (w=1.5, no UB)        -- inflated heuristic only" << std::endl;
    std::cout << "  D: Norm+ERCA* (w=1.5, with UB) -- FULL strategy (norm + UB + inflation)" << std::endl;
    std::cout << std::endl;

    struct TestConfig {
        int gridSize;
        int numRes;
        double extraEdge;
        std::string desc;
    };

    std::vector<TestConfig> configs = {
        {15, 2, 0.3, "15x15/2res"},
        {20, 2, 0.3, "20x20/2res"},
        {25, 2, 0.3, "25x25/2res"},
        {30, 2, 0.3, "30x30/2res"},
        {20, 2, 0.6, "20x20/2res/dense"},
        {25, 2, 0.6, "25x25/2res/dense"},
        {30, 2, 0.6, "30x30/2res/dense"},
        {20, 3, 0.3, "20x20/3res"},
        {25, 3, 0.3, "25x25/3res"},
    };

    double tightness[] = {1.1, 1.2, 1.5, 2.0, 3.0};
    int numTightness = 5;

    std::vector<TestRow> allRows;

    for (auto& cfg : configs) {
        std::cout << "=== " << cfg.desc << " ===" << std::endl;

        Roadmap graph = generateGraph(cfg.gridSize, cfg.numRes, cfg.extraEdge);
        long nNodes = graph.GetNumberOfNodes();
        long nEdges = graph.GetNumberOfEdges();
        long source = 1;
        long target = nNodes;

        std::cout << "  N=" << nNodes << " E=" << nEdges << std::endl;

        std::vector<long> minResCosts(cfg.numRes);
        bool reachable = true;
        for (int r = 0; r < cfg.numRes; r++) {
            DijkstraScan dijk;
            dijk.SetGraphPtr(&graph);
            dijk.Search(target, r + 1);
            long cost = dijk.GetCost(source);
            if (cost <= 0) { reachable = false; break; }
            minResCosts[r] = cost;
        }
        if (!reachable) { std::cout << "  Skip\n"; continue; }

        for (int t = 0; t < numTightness; t++) {
            double mult = tightness[t];
            std::vector<long> resLimits;
            for (int r = 0; r < cfg.numRes; r++) {
                resLimits.push_back(static_cast<long>(minResCosts[r] * mult));
            }

            std::cout << "  T=" << mult << "x: " << std::flush;

            TestRow row;
            row.config = cfg.desc;
            row.gridSize = cfg.gridSize;
            row.numRes = cfg.numRes;
            row.tightness = mult;
            row.nNodes = nNodes;
            row.nEdges = nEdges;

            // A: Exact ERCA*
            row.baseline = runERCA(graph, source, target, resLimits, 1.0, 30.0);

            // B: Norm+ERCA* (exact, with UB)
            row.withUB = runERCAWithNormUB(graph, source, target, resLimits, 1.0, 30.0);

            // C: Inflated ERCA* (w=1.5, no UB)
            row.inflated = runERCA(graph, source, target, resLimits, 1.5, 30.0);

            // D: Full strategy: Norm+UB+Inflation
            row.inflatedUB = runERCAWithNormUB(graph, source, target, resLimits, 1.5, 30.0);

            allRows.push_back(row);

            // Quick summary
            std::cout << "A=" << row.baseline.expanded
                      << " B=" << row.withUB.expanded
                      << "(UBp=" << row.withUB.prunedUpperBound << ")"
                      << " C=" << row.inflated.expanded
                      << " D=" << row.inflatedUB.expanded
                      << "(UBp=" << row.inflatedUB.prunedUpperBound << ")";
            if (row.inflatedUB.normFeasible && row.baseline.expanded > 0) {
                double reduc = 1.0 - (double)row.inflatedUB.expanded / row.baseline.expanded;
                std::cout << " Redu=" << std::fixed << std::setprecision(1) << reduc * 100 << "%";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    // === Summary Table ===
    std::cout << "\n=== SUMMARY: Expanded Nodes Comparison ===" << std::endl;
    std::cout << std::setw(20) << "Config"
              << std::setw(6) << "Tight"
              << std::setw(10) << "A:exact"
              << std::setw(10) << "B:+UB"
              << std::setw(10) << "C:w=1.5"
              << std::setw(10) << "D:UB+w"
              << std::setw(8) << "D/A %"
              << std::setw(8) << "Norm"
              << std::setw(8) << "UBcost"
              << std::setw(8) << "OPT"
              << std::endl;

    for (auto& row : allRows) {
        double ratio = (row.baseline.expanded > 0)
            ? (double)row.inflatedUB.expanded / row.baseline.expanded * 100
            : 0;
        std::cout << std::setw(20) << row.config
                  << std::setw(6) << std::fixed << std::setprecision(1) << row.tightness
                  << std::setw(10) << row.baseline.expanded
                  << std::setw(10) << row.withUB.expanded
                  << std::setw(10) << row.inflated.expanded
                  << std::setw(10) << row.inflatedUB.expanded
                  << std::setw(8) << std::setprecision(1) << ratio
                  << std::setw(8) << (row.inflatedUB.normFeasible ? "Y" : "N")
                  << std::setw(8) << row.inflatedUB.ubCost
                  << std::setw(8) << row.baseline.cost
                  << std::endl;
    }

    // === Timing Table ===
    std::cout << "\n=== SUMMARY: Time Comparison (ms) ===" << std::endl;
    std::cout << std::setw(20) << "Config"
              << std::setw(6) << "Tight"
              << std::setw(10) << "A:exact"
              << std::setw(10) << "B:total"
              << std::setw(10) << "B:norm"
              << std::setw(10) << "C:w=1.5"
              << std::setw(10) << "D:total"
              << std::setw(10) << "D:norm"
              << std::setw(8) << "D:cost"
              << std::setw(8) << "A:cost"
              << std::endl;

    for (auto& row : allRows) {
        std::cout << std::setw(20) << row.config
                  << std::setw(6) << std::fixed << std::setprecision(1) << row.tightness
                  << std::setw(10) << std::setprecision(2) << row.baseline.totalTimeMs
                  << std::setw(10) << row.withUB.totalTimeMs
                  << std::setw(10) << row.withUB.normTimeMs
                  << std::setw(10) << row.inflated.totalTimeMs
                  << std::setw(10) << row.inflatedUB.totalTimeMs
                  << std::setw(10) << row.inflatedUB.normTimeMs
                  << std::setw(8) << row.inflatedUB.cost
                  << std::setw(8) << row.baseline.cost
                  << std::endl;
    }

    // === Pruning Detail ===
    std::cout << "\n=== PRUNING DETAIL (Method D: Norm+UB+w=1.5) ===" << std::endl;
    std::cout << std::setw(20) << "Config"
              << std::setw(6) << "Tight"
              << std::setw(8) << "Expand"
              << std::setw(8) << "UBPrn"
              << std::setw(8) << "ResPrn"
              << std::setw(8) << "DomPrn"
              << std::setw(8) << "UBcost"
              << std::setw(8) << "Dcost"
              << std::setw(10) << "UBtight%"
              << std::endl;

    for (auto& row : allRows) {
        double ubTightness = 0;
        if (row.inflatedUB.ubCost > 0 && row.baseline.cost > 0) {
            ubTightness = (double)row.inflatedUB.ubCost / row.baseline.cost * 100;
        }
        std::cout << std::setw(20) << row.config
                  << std::setw(6) << std::fixed << std::setprecision(1) << row.tightness
                  << std::setw(8) << row.inflatedUB.expanded
                  << std::setw(8) << row.inflatedUB.prunedUpperBound
                  << std::setw(8) << row.inflatedUB.prunedResource
                  << std::setw(8) << row.inflatedUB.prunedDominance
                  << std::setw(8) << row.inflatedUB.ubCost
                  << std::setw(8) << row.inflatedUB.cost
                  << std::setw(10) << std::setprecision(1) << ubTightness
                  << std::endl;
    }

    // Export CSV
    std::ofstream fout("pruning_analysis.csv");
    if (fout.is_open()) {
        fout << "config,gridSize,numRes,tightness,nodes,edges,"
             << "A_time_ms,A_expanded,A_cost,"
             << "B_time_ms,B_norm_ms,B_expanded,B_UBprune,B_cost,B_normOK,B_UBcost,"
             << "C_time_ms,C_expanded,C_cost,"
             << "D_time_ms,D_norm_ms,D_expanded,D_UBprune,D_cost,D_normOK,D_UBcost" << std::endl;
        for (auto& row : allRows) {
            fout << "\"" << row.config << "\","
                 << row.gridSize << "," << row.numRes << "," << row.tightness << ","
                 << row.nNodes << "," << row.nEdges << ","
                 << row.baseline.totalTimeMs << "," << row.baseline.expanded << "," << row.baseline.cost << ","
                 << row.withUB.totalTimeMs << "," << row.withUB.normTimeMs << ","
                 << row.withUB.expanded << "," << row.withUB.prunedUpperBound << ","
                 << row.withUB.cost << "," << (row.withUB.normFeasible ? 1 : 0) << "," << row.withUB.ubCost << ","
                 << row.inflated.totalTimeMs << "," << row.inflated.expanded << "," << row.inflated.cost << ","
                 << row.inflatedUB.totalTimeMs << "," << row.inflatedUB.normTimeMs << ","
                 << row.inflatedUB.expanded << "," << row.inflatedUB.prunedUpperBound << ","
                 << row.inflatedUB.cost << "," << (row.inflatedUB.normFeasible ? 1 : 0) << ","
                 << row.inflatedUB.ubCost << std::endl;
        }
        fout.close();
        std::cout << "\nResults saved to pruning_analysis.csv" << std::endl;
    }

    return 0;
}
