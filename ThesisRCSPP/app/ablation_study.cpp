#include "graph/Roadmap.h"
#include "algorithm/ERCA.h"
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

// ============================================================
// Graph generation (same as pruning_analysis)
// ============================================================
Roadmap generateGraph(int gridSize, int numResources, double extraEdgeFraction, unsigned seed = 42) {
    std::mt19937 rng(seed);
    size_t costDim = 1 + numResources;
    Roadmap graph;
    int numNodes = gridSize * gridSize;
    graph.Init(numNodes, costDim);
    for (int i = 1; i <= numNodes; i++) graph.AddNode(i);

    std::uniform_int_distribution<long> costDist(50, 500);
    std::uniform_int_distribution<long> resDist(20, 300);
    std::uniform_real_distribution<double> edgeProb(0, 1);

    auto addEdgePair = [&](long from, long to) {
        CostVector cost(0, costDim);
        cost[0] = costDist(rng);
        for (int r = 0; r < numResources; r++) cost[r + 1] = resDist(rng);
        graph.AddEdge(from, to, cost);
        graph.AddEdge(to, from, cost);
    };

    for (int y = 0; y < gridSize; y++)
        for (int x = 0; x < gridSize; x++) {
            long nid = y * gridSize + x + 1;
            if (x + 1 < gridSize) addEdgePair(nid, nid + 1);
            if (y + 1 < gridSize) addEdgePair(nid, nid + gridSize);
        }
    for (int y = 0; y < gridSize; y++)
        for (int x = 0; x < gridSize; x++) {
            long nid = y * gridSize + x + 1;
            if (x + 1 < gridSize && y + 1 < gridSize && edgeProb(rng) < extraEdgeFraction)
                addEdgePair(nid, (y + 1) * gridSize + (x + 1) + 1);
            if (x > 0 && y + 1 < gridSize && edgeProb(rng) < extraEdgeFraction * 0.5)
                addEdgePair(nid, (y + 1) * gridSize + (x - 1) + 1);
        }
    return graph;
}

// ============================================================
// Result structure for one ablation method
// ============================================================
struct AblationResult {
    std::string method;
    double totalTimeMs;
    double normTimeMs;
    long expanded;
    long generated;
    long prunedUB;
    long cost;
    bool solved;
    bool normOK;
    long ubCost;
};

// ============================================================
// Run a single ablation configuration
// ============================================================
// useNorm:     run Normalizer to find upper bound
// maxNormIter: 1 = no weight adjustment, 5 = full weight adjustment
// useUB:       pass upper bound to ERCA* for pruning
// w:           heuristic inflation factor (1.0 = exact)
AblationResult runAblation(Roadmap& graph, long source, long target,
                           const std::vector<long>& resLimits,
                           const std::string& name,
                           bool useNorm, int maxNormIter,
                           bool useUB, double w,
                           double timeLimit) {
    AblationResult r;
    r.method = name;
    r.normTimeMs = 0;
    r.normOK = false;
    r.ubCost = -1;

    auto t0 = std::chrono::steady_clock::now();

    CostVector upperBound;
    if (useNorm) {
        Normalizer norm;
        norm.SetGraph(&graph);
        norm.SetResourceLimits(resLimits);
        NormResult normRes = norm.ComputeUpperBoundERCA(source, target, maxNormIter);
        auto tNorm = std::chrono::steady_clock::now();
        r.normTimeMs = std::chrono::duration<double>(tNorm - t0).count() * 1000;
        r.normOK = normRes.feasible;
        if (normRes.feasible) {
            upperBound = normRes.upperBound;
            r.ubCost = normRes.upperBound[0];
        }
    }

    ERCA erca;
    erca.SetGraph(&graph);
    erca.SetResourceLimits(resLimits);
    erca.SetHeuInflateRate(w);
    if (useNorm && useUB && r.normOK) {
        erca.SetUpperBound(upperBound);
    }
    erca.Search(source, target, timeLimit);

    auto t1 = std::chrono::steady_clock::now();
    SearchResult res = erca.GetResult();
    r.totalTimeMs = std::chrono::duration<double>(t1 - t0).count() * 1000;
    r.expanded = res.nExpanded;
    r.generated = res.nGenerated;
    r.prunedUB = res.nPrunedByUpperBound;
    r.solved = !res.paths.empty();
    r.cost = r.solved ? res.costs.begin()->second[0] : -1;

    // If ERCA* didn't find solution but normalizer did, report normalizer's cost
    if (!r.solved && r.normOK) {
        r.solved = true;
        r.cost = r.ubCost;
    }

    return r;
}

// ============================================================
// Main
// ============================================================
int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "     Ablation Study: Component Analysis" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "Ablation configurations:" << std::endl;
    std::cout << "  Base  : ERCA* (w=1.0, no UB)                     -- baseline" << std::endl;
    std::cout << "  Full  : Norm(5iter) + UB + w=1.5                  -- complete strategy" << std::endl;
    std::cout << "  -Infl : Norm(5iter) + UB + w=1.0                  -- remove inflation" << std::endl;
    std::cout << "  -UB   : Norm(5iter) + w=1.5 (no UB pruning)       -- remove UB pruning" << std::endl;
    std::cout << "  -WA   : Norm(1iter) + UB + w=1.5                  -- remove weight adj" << std::endl;
    std::cout << "  -Norm : w=1.5 only (no normalization)             -- remove normalization" << std::endl;
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
        {20, 2, 0.6, "20x20/2res/d"},
        {25, 2, 0.6, "25x25/2res/d"},
        {30, 2, 0.6, "30x30/2res/d"},
        {20, 3, 0.3, "20x20/3res"},
        {25, 3, 0.3, "25x25/3res"},
    };

    double tightness[] = {1.2, 1.5, 2.0, 3.0};
    int numTightness = 4;

    // Collect all rows: one row = one (config, tightness) with 6 ablation results
    struct AblationRow {
        std::string config;
        double tight;
        long nNodes, nEdges;
        AblationResult base, full, noInfl, noUB, noWA, noNorm;
    };
    std::vector<AblationRow> allRows;

    for (auto& cfg : configs) {
        std::cout << "=== " << cfg.desc << " ===" << std::endl;
        Roadmap graph = generateGraph(cfg.gridSize, cfg.numRes, cfg.extraEdge);
        long N = graph.GetNumberOfNodes();
        long E = graph.GetNumberOfEdges();
        long source = 1, target = N;

        std::vector<long> minRes(cfg.numRes);
        bool ok = true;
        for (int r = 0; r < cfg.numRes; r++) {
            DijkstraScan d; d.SetGraphPtr(&graph); d.Search(target, r + 1);
            long c = d.GetCost(source);
            if (c <= 0) { ok = false; break; }
            minRes[r] = c;
        }
        if (!ok) { std::cout << "  Skip\n"; continue; }

        for (int t = 0; t < numTightness; t++) {
            double mult = tightness[t];
            std::vector<long> limits;
            for (int r = 0; r < cfg.numRes; r++)
                limits.push_back(static_cast<long>(minRes[r] * mult));

            std::cout << "  T=" << mult << "x: " << std::flush;

            AblationRow row;
            row.config = cfg.desc;
            row.tight = mult;
            row.nNodes = N;
            row.nEdges = E;

            //                                     useNorm  iter  useUB   w
            row.base   = runAblation(graph, source, target, limits,
                "Base",   false, 0, false, 1.0, 30.0);
            row.full   = runAblation(graph, source, target, limits,
                "Full",   true,  5, true,  1.5, 30.0);
            row.noInfl = runAblation(graph, source, target, limits,
                "-Infl",  true,  5, true,  1.0, 30.0);
            row.noUB   = runAblation(graph, source, target, limits,
                "-UB",    true,  5, false, 1.5, 30.0);
            row.noWA   = runAblation(graph, source, target, limits,
                "-WA",    true,  1, true,  1.5, 30.0);
            row.noNorm = runAblation(graph, source, target, limits,
                "-Norm",  false, 0, false, 1.5, 30.0);

            allRows.push_back(row);

            std::cout << "Base=" << row.base.expanded
                      << " Full=" << row.full.expanded
                      << " -Infl=" << row.noInfl.expanded
                      << " -UB=" << row.noUB.expanded
                      << " -WA=" << row.noWA.expanded
                      << " -Norm=" << row.noNorm.expanded
                      << std::endl;
        }
        std::cout << std::endl;
    }

    // ============================================================
    // Summary Table 1: Expanded Nodes
    // ============================================================
    std::cout << "\n========================================" << std::endl;
    std::cout << "  TABLE 1: Expanded Nodes (lower=better)" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::setw(15) << "Config"
              << std::setw(6) << "Tight"
              << std::setw(8) << "Base"
              << std::setw(8) << "Full"
              << std::setw(8) << "-Infl"
              << std::setw(8) << "-UB"
              << std::setw(8) << "-WA"
              << std::setw(8) << "-Norm"
              << std::endl;
    std::cout << std::string(63, '-') << std::endl;

    for (auto& row : allRows) {
        std::cout << std::setw(15) << row.config
                  << std::setw(6) << std::fixed << std::setprecision(1) << row.tight
                  << std::setw(8) << row.base.expanded
                  << std::setw(8) << row.full.expanded
                  << std::setw(8) << row.noInfl.expanded
                  << std::setw(8) << row.noUB.expanded
                  << std::setw(8) << row.noWA.expanded
                  << std::setw(8) << row.noNorm.expanded
                  << std::endl;
    }

    // ============================================================
    // Summary Table 2: Total Time (ms)
    // ============================================================
    std::cout << "\n========================================" << std::endl;
    std::cout << "  TABLE 2: Total Time ms (lower=better)" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::setw(15) << "Config"
              << std::setw(6) << "Tight"
              << std::setw(10) << "Base"
              << std::setw(10) << "Full"
              << std::setw(10) << "-Infl"
              << std::setw(10) << "-UB"
              << std::setw(10) << "-WA"
              << std::setw(10) << "-Norm"
              << std::endl;
    std::cout << std::string(71, '-') << std::endl;

    for (auto& row : allRows) {
        std::cout << std::setw(15) << row.config
                  << std::setw(6) << std::fixed << std::setprecision(1) << row.tight
                  << std::setw(10) << std::setprecision(2) << row.base.totalTimeMs
                  << std::setw(10) << row.full.totalTimeMs
                  << std::setw(10) << row.noInfl.totalTimeMs
                  << std::setw(10) << row.noUB.totalTimeMs
                  << std::setw(10) << row.noWA.totalTimeMs
                  << std::setw(10) << row.noNorm.totalTimeMs
                  << std::endl;
    }

    // ============================================================
    // Summary Table 3: Solution Quality (cost)
    // ============================================================
    std::cout << "\n========================================" << std::endl;
    std::cout << "  TABLE 3: Solution Cost (lower=better)" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::setw(15) << "Config"
              << std::setw(6) << "Tight"
              << std::setw(8) << "Base"
              << std::setw(8) << "Full"
              << std::setw(8) << "-Infl"
              << std::setw(8) << "-UB"
              << std::setw(8) << "-WA"
              << std::setw(8) << "-Norm"
              << std::endl;
    std::cout << std::string(63, '-') << std::endl;

    for (auto& row : allRows) {
        std::cout << std::setw(15) << row.config
                  << std::setw(6) << std::fixed << std::setprecision(1) << row.tight
                  << std::setw(8) << row.base.cost
                  << std::setw(8) << row.full.cost
                  << std::setw(8) << row.noInfl.cost
                  << std::setw(8) << row.noUB.cost
                  << std::setw(8) << row.noWA.cost
                  << std::setw(8) << row.noNorm.cost
                  << std::endl;
    }

    // ============================================================
    // Summary Table 4: Normalization Details
    // ============================================================
    std::cout << "\n========================================" << std::endl;
    std::cout << "  TABLE 4: Normalization Success & UB" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::setw(15) << "Config"
              << std::setw(6) << "Tight"
              << std::setw(8) << "F:Norm"
              << std::setw(8) << "F:UB"
              << std::setw(8) << "F:UBp"
              << std::setw(8) << "-I:Norm"
              << std::setw(8) << "-I:UBp"
              << std::setw(8) << "-W:Norm"
              << std::setw(8) << "-W:UBp"
              << std::setw(8) << "OPT"
              << std::endl;
    std::cout << std::string(79, '-') << std::endl;

    for (auto& row : allRows) {
        std::cout << std::setw(15) << row.config
                  << std::setw(6) << std::fixed << std::setprecision(1) << row.tight
                  << std::setw(8) << (row.full.normOK ? "Y" : "N")
                  << std::setw(8) << row.full.ubCost
                  << std::setw(8) << row.full.prunedUB
                  << std::setw(8) << (row.noInfl.normOK ? "Y" : "N")
                  << std::setw(8) << row.noInfl.prunedUB
                  << std::setw(8) << (row.noWA.normOK ? "Y" : "N")
                  << std::setw(8) << row.noWA.prunedUB
                  << std::setw(8) << row.base.cost
                  << std::endl;
    }

    // ============================================================
    // Component contribution analysis
    // ============================================================
    std::cout << "\n========================================" << std::endl;
    std::cout << "  TABLE 5: Component Contribution" << std::endl;
    std::cout << "  (Expanded nodes reduction vs Base)" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::setw(15) << "Config"
              << std::setw(6) << "Tight"
              << std::setw(10) << "Full%"
              << std::setw(10) << "Infl"
              << std::setw(10) << "UB"
              << std::setw(10) << "WeightAdj"
              << std::setw(10) << "Norm"
              << std::endl;
    std::cout << std::string(61, '-') << std::endl;

    for (auto& row : allRows) {
        if (row.base.expanded == 0) continue;
        double baseExp = (double)row.base.expanded;

        // Full strategy total reduction
        double fullRedu = (1.0 - row.full.expanded / baseExp) * 100;

        // Inflation contribution: Full vs -Infl (what inflation adds)
        // If -Infl has more expanded than Full, inflation helps
        double inflContr = 0;
        if (row.noInfl.expanded > 0)
            inflContr = (1.0 - (double)row.full.expanded / row.noInfl.expanded) * 100;
        else if (row.full.expanded == 0)
            inflContr = 0; // both 0

        // UB contribution: Full vs -UB
        double ubContr = 0;
        if (row.noUB.expanded > 0)
            ubContr = (1.0 - (double)row.full.expanded / row.noUB.expanded) * 100;

        // Weight adjustment contribution: Full vs -WA
        double waContr = 0;
        if (row.noWA.expanded > 0)
            waContr = (1.0 - (double)row.full.expanded / row.noWA.expanded) * 100;

        // Normalization contribution: Full vs -Norm
        double normContr = 0;
        if (row.noNorm.expanded > 0)
            normContr = (1.0 - (double)row.full.expanded / row.noNorm.expanded) * 100;

        std::cout << std::setw(15) << row.config
                  << std::setw(6) << std::fixed << std::setprecision(1) << row.tight
                  << std::setw(10) << std::setprecision(1) << fullRedu
                  << std::setw(10) << inflContr
                  << std::setw(10) << ubContr
                  << std::setw(10) << waContr
                  << std::setw(10) << normContr
                  << std::endl;
    }

    // ============================================================
    // Export CSV
    // ============================================================
    std::ofstream fout("ablation_results.csv");
    if (fout.is_open()) {
        fout << "config,tightness,nodes,edges,"
             << "base_time,base_exp,base_cost,"
             << "full_time,full_norm_ms,full_exp,full_cost,full_normOK,full_UBcost,full_UBprune,"
             << "noInfl_time,noInfl_exp,noInfl_cost,noInfl_normOK,noInfl_UBprune,"
             << "noUB_time,noUB_exp,noUB_cost,noUB_normOK,"
             << "noWA_time,noWA_exp,noWA_cost,noWA_normOK,noWA_UBprune,"
             << "noNorm_time,noNorm_exp,noNorm_cost" << std::endl;
        for (auto& row : allRows) {
            fout << "\"" << row.config << "\"," << row.tight << "," << row.nNodes << "," << row.nEdges << ","
                 << row.base.totalTimeMs << "," << row.base.expanded << "," << row.base.cost << ","
                 << row.full.totalTimeMs << "," << row.full.normTimeMs << "," << row.full.expanded << ","
                 << row.full.cost << "," << (row.full.normOK?1:0) << "," << row.full.ubCost << "," << row.full.prunedUB << ","
                 << row.noInfl.totalTimeMs << "," << row.noInfl.expanded << "," << row.noInfl.cost << ","
                 << (row.noInfl.normOK?1:0) << "," << row.noInfl.prunedUB << ","
                 << row.noUB.totalTimeMs << "," << row.noUB.expanded << "," << row.noUB.cost << ","
                 << (row.noUB.normOK?1:0) << ","
                 << row.noWA.totalTimeMs << "," << row.noWA.expanded << "," << row.noWA.cost << ","
                 << (row.noWA.normOK?1:0) << "," << row.noWA.prunedUB << ","
                 << row.noNorm.totalTimeMs << "," << row.noNorm.expanded << "," << row.noNorm.cost
                 << std::endl;
        }
        fout.close();
        std::cout << "\nResults saved to ablation_results.csv" << std::endl;
    }

    return 0;
}
