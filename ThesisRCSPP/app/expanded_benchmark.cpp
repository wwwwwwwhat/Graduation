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
#include <numeric>
#include <algorithm>

using namespace rcspp;

// ============================================================
// Graph generation with seed parameter
// ============================================================
Roadmap generateGraph(int gridSize, int numResources, double extraEdgeFraction, unsigned seed) {
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
// Run ERCA* with given parameters, return timing and result
// ============================================================
struct BenchResult {
    double totalTimeMs = 0;
    double searchTimeMs = 0;
    long expanded = 0;
    long generated = 0;
    long cost = -1;
    bool solved = false;
    bool timeout = false;
};

BenchResult runERCA(Roadmap& graph, long source, long target,
                    const std::vector<long>& resLimits,
                    double w, const CostVector* upperBound,
                    double timeLimit) {
    BenchResult br;
    auto t0 = std::chrono::steady_clock::now();

    ERCA erca;
    erca.SetGraph(&graph);
    erca.SetResourceLimits(resLimits);
    erca.SetHeuInflateRate(w);
    if (upperBound) erca.SetUpperBound(*upperBound);
    erca.Search(source, target, timeLimit);

    auto t1 = std::chrono::steady_clock::now();
    SearchResult res = erca.GetResult();
    br.totalTimeMs = std::chrono::duration<double>(t1 - t0).count() * 1000;
    br.searchTimeMs = res.rtSearch * 1000;
    br.expanded = res.nExpanded;
    br.generated = res.nGenerated;
    br.timeout = res.timeout;
    br.solved = !res.paths.empty();
    if (br.solved) br.cost = res.costs.begin()->second[0];
    return br;
}

// ============================================================
// Main
// ============================================================
int main() {
    std::cout << "================================================================" << std::endl;
    std::cout << "  Expanded Benchmark: More Data + Iteration Analysis" << std::endl;
    std::cout << "================================================================" << std::endl;

    struct TestConfig {
        int gridSize;
        int numRes;
        double extraEdge;
        std::string desc;
    };

    // More configurations for comprehensive testing
    std::vector<TestConfig> configs = {
        {10, 2, 0.3, "10x10/2r"},
        {15, 2, 0.3, "15x15/2r"},
        {20, 2, 0.3, "20x20/2r"},
        {25, 2, 0.3, "25x25/2r"},
        {30, 2, 0.3, "30x30/2r"},
        {35, 2, 0.3, "35x35/2r"},
        {15, 2, 0.6, "15x15/2r/d"},
        {20, 2, 0.6, "20x20/2r/d"},
        {25, 2, 0.6, "25x25/2r/d"},
        {30, 2, 0.6, "30x30/2r/d"},
        {15, 3, 0.3, "15x15/3r"},
        {20, 3, 0.3, "20x20/3r"},
        {25, 3, 0.3, "25x25/3r"},
    };

    std::vector<unsigned> seeds = {42, 123, 7, 2024, 999};
    double tightness[] = {1.2, 1.5, 2.0, 3.0, 5.0};
    int numTightness = 5;
    int maxNormIter = 5;

    // ============================================================
    // Part 1: Multi-seed aggregate results (speedup ratios)
    // ============================================================
    struct AggRow {
        std::string config;
        double tight;
        long nNodes, nEdges;
        // averaged across seeds
        double baseTimeAvg, fullTimeAvg;
        double baseExpAvg, fullExpAvg;
        double baseCostAvg, fullCostAvg;
        double normTimeAvg; // normalization preprocessing time
        double speedup;     // base_time / full_time
        double nodeRedu;    // 1 - full_exp / base_exp
        double costRatio;   // full_cost / base_cost (1.0 = optimal)
        int nSeeds;         // how many seeds succeeded
    };
    std::vector<AggRow> aggRows;

    // ============================================================
    // Part 2: Per-iteration analysis (UB quality across iterations)
    // ============================================================
    struct IterRow {
        std::string config;
        double tight;
        int iteration;
        // averaged across seeds
        double ubCostAvg;     // UB cost at this iteration
        double optCostAvg;    // optimal cost
        double costRatioAvg;  // UB / optimal
        double feasibleRate;  // fraction of seeds where this iter found feasible solution
        double cumTimeMsAvg;  // cumulative normalization time
        // when using this iter's UB for ERCA*:
        double ercaExpAvg;    // expanded nodes
        double ercaTimeAvg;   // total time (norm + ERCA)
        double totalSpeedupAvg; // base_time / (norm_time + erca_time)
    };
    std::vector<IterRow> iterRows;

    // CSV output
    std::ofstream csvMain("expanded_benchmark_main.csv");
    csvMain << "config,tightness,seed,nodes,edges,"
            << "base_time_ms,base_exp,base_cost,"
            << "full_time_ms,full_exp,full_cost,full_norm_ms,"
            << "speedup,node_reduction,cost_ratio" << std::endl;

    std::ofstream csvIter("expanded_benchmark_iter.csv");
    csvIter << "config,tightness,seed,iteration,"
            << "found_solution,feasible_original,ub_cost,optimal_cost,cost_ratio,"
            << "iter_time_ms,cum_time_ms,"
            << "erca_with_ub_exp,erca_with_ub_time_ms,total_speedup" << std::endl;

    for (auto& cfg : configs) {
        std::cout << "\n=== " << cfg.desc << " ===" << std::endl;

        // Accumulators for aggregation
        std::vector<double> baseTimesAll, fullTimesAll;
        std::vector<double> baseExpsAll, fullExpsAll;
        std::vector<double> baseCostsAll, fullCostsAll;
        std::vector<double> normTimesAll;

        // Per-tightness accumulators
        for (int ti = 0; ti < numTightness; ti++) {
            double mult = tightness[ti];
            std::vector<double> baseTimes, fullTimes, baseExps, fullExps;
            std::vector<double> baseCosts, fullCosts, normTimes;

            // Per-iteration accumulators (across seeds)
            // iterAccum[iter] = {ubCosts, optCosts, feasibleCount, cumTimes, ercaExps, ercaTimes}
            struct IterAccum {
                std::vector<double> ubCosts, optCosts, cumTimes;
                std::vector<double> ercaExps, ercaTimes, totalSpeedups;
                int feasibleCount = 0;
                int totalCount = 0;
            };
            std::vector<IterAccum> iterAccums(maxNormIter);

            long lastN = 0, lastE = 0;

            for (auto seed : seeds) {
                Roadmap graph = generateGraph(cfg.gridSize, cfg.numRes, cfg.extraEdge, seed);
                long N = graph.GetNumberOfNodes();
                long E = graph.GetNumberOfEdges();
                lastN = N; lastE = E;
                long source = 1, target = N;

                // Compute minimum resource costs
                std::vector<long> minRes(cfg.numRes);
                bool ok = true;
                for (int r = 0; r < cfg.numRes; r++) {
                    DijkstraScan d;
                    d.SetGraphPtr(&graph);
                    d.Search(target, r + 1);
                    long c = d.GetCost(source);
                    if (c <= 0) { ok = false; break; }
                    minRes[r] = c;
                }
                if (!ok) continue;

                std::vector<long> limits;
                for (int r = 0; r < cfg.numRes; r++)
                    limits.push_back(static_cast<long>(minRes[r] * mult));

                // (A) Baseline: exact ERCA* (w=1.0)
                BenchResult base = runERCA(graph, source, target, limits, 1.0, nullptr, 60.0);
                if (!base.solved || base.timeout) continue;

                // (B) Full strategy: Norm(5iter) + UB + w=1.5
                auto normStart = std::chrono::steady_clock::now();
                Normalizer norm;
                norm.SetGraph(&graph);
                norm.SetResourceLimits(limits);
                NormResult normRes = norm.ComputeUpperBoundERCA_Detailed(source, target, maxNormIter);
                auto normEnd = std::chrono::steady_clock::now();
                double normMs = std::chrono::duration<double>(normEnd - normStart).count() * 1000;

                BenchResult full;
                if (normRes.feasible) {
                    full = runERCA(graph, source, target, limits, 1.5, &normRes.upperBound, 60.0);
                } else {
                    full = runERCA(graph, source, target, limits, 1.5, nullptr, 60.0);
                }
                full.totalTimeMs += normMs; // include normalization time

                // If ERCA* with inflation didn't find a path but norm did,
                // use normalization result (norm already found optimal)
                long fullCost = full.solved ? full.cost : -1;
                if (!full.solved && normRes.feasible) {
                    fullCost = normRes.upperBound[0];
                }

                baseTimes.push_back(base.totalTimeMs);
                fullTimes.push_back(full.totalTimeMs);
                baseExps.push_back(base.expanded);
                fullExps.push_back(full.expanded);
                baseCosts.push_back(base.cost);
                fullCosts.push_back(fullCost);
                normTimes.push_back(normMs);

                // CSV main row
                double sp = (full.totalTimeMs > 0) ? base.totalTimeMs / full.totalTimeMs : 0;
                double nr = (base.expanded > 0) ? (1.0 - (double)full.expanded / base.expanded) : 0;
                double cr = (base.cost > 0 && fullCost > 0) ? (double)fullCost / base.cost : -1;
                csvMain << "\"" << cfg.desc << "\"," << mult << "," << seed << ","
                        << N << "," << E << ","
                        << base.totalTimeMs << "," << base.expanded << "," << base.cost << ","
                        << full.totalTimeMs << "," << full.expanded << "," << fullCost << "," << normMs << ","
                        << sp << "," << nr << "," << cr << std::endl;

                // Per-iteration analysis
                for (size_t it = 0; it < normRes.iterDetails.size(); it++) {
                    auto& d = normRes.iterDetails[it];
                    iterAccums[it].totalCount++;

                    if (d.foundSolution && d.feasibleOnOriginal) {
                        iterAccums[it].feasibleCount++;
                        iterAccums[it].ubCosts.push_back(d.cost);
                        iterAccums[it].optCosts.push_back(base.cost);
                        iterAccums[it].cumTimes.push_back(d.cumTimeMs);

                        // Run ERCA* using this iteration's UB
                        CostVector iterUB(0, graph.GetCostDim());
                        // Reconstruct cost from path on original graph
                        // We already have d.cost as primary cost
                        // For the UB, we need the full cost vector
                        // Let's compute it from the normRes if this is the final feasible iteration
                        // Actually, we should reconstruct by re-running with limited iterations
                        // Simpler: just use d.cost as UB[0]
                        iterUB[0] = d.cost;
                        for (size_t r = 0; r < (size_t)cfg.numRes; r++)
                            iterUB[r + 1] = limits[r]; // keep resource limits as-is

                        BenchResult iterERCA = runERCA(graph, source, target, limits, 1.5, &iterUB, 60.0);
                        double iterTotal = d.cumTimeMs + iterERCA.totalTimeMs;
                        double iterSpeedup = (iterTotal > 0) ? base.totalTimeMs / iterTotal : 0;

                        iterAccums[it].ercaExps.push_back(iterERCA.expanded);
                        iterAccums[it].ercaTimes.push_back(iterERCA.totalTimeMs);
                        iterAccums[it].totalSpeedups.push_back(iterSpeedup);

                        csvIter << "\"" << cfg.desc << "\"," << mult << "," << seed << ","
                                << (it + 1) << ","
                                << 1 << "," << 1 << ","
                                << d.cost << "," << base.cost << ","
                                << ((base.cost > 0) ? (double)d.cost / base.cost : -1) << ","
                                << d.iterTimeMs << "," << d.cumTimeMs << ","
                                << iterERCA.expanded << "," << iterERCA.totalTimeMs << ","
                                << iterSpeedup << std::endl;
                    } else {
                        csvIter << "\"" << cfg.desc << "\"," << mult << "," << seed << ","
                                << (it + 1) << ","
                                << (d.foundSolution ? 1 : 0) << "," << 0 << ","
                                << d.cost << "," << base.cost << ",-1,"
                                << d.iterTimeMs << "," << d.cumTimeMs << ","
                                << "-1,-1,-1" << std::endl;
                    }
                }
            }

            // Aggregate
            if (baseTimes.empty()) continue;
            AggRow agg;
            agg.config = cfg.desc;
            agg.tight = mult;
            agg.nNodes = lastN;
            agg.nEdges = lastE;
            agg.nSeeds = baseTimes.size();

            auto avg = [](const std::vector<double>& v) {
                return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / v.size();
            };

            agg.baseTimeAvg = avg(baseTimes);
            agg.fullTimeAvg = avg(fullTimes);
            agg.baseExpAvg = avg(baseExps);
            agg.fullExpAvg = avg(fullExps);
            agg.baseCostAvg = avg(baseCosts);
            agg.fullCostAvg = avg(fullCosts);
            agg.normTimeAvg = avg(normTimes);
            agg.speedup = (agg.fullTimeAvg > 0) ? agg.baseTimeAvg / agg.fullTimeAvg : 0;
            agg.nodeRedu = (agg.baseExpAvg > 0) ? (1.0 - agg.fullExpAvg / agg.baseExpAvg) : 0;
            // Filter out -1 values for cost ratio
            {
                std::vector<double> validRatios;
                for (size_t si = 0; si < fullCosts.size(); si++) {
                    if (fullCosts[si] > 0 && baseCosts[si] > 0)
                        validRatios.push_back(fullCosts[si] / baseCosts[si]);
                }
                agg.costRatio = validRatios.empty() ? -1 :
                    std::accumulate(validRatios.begin(), validRatios.end(), 0.0) / validRatios.size();
            }
            aggRows.push_back(agg);

            // Per-iteration aggregate
            for (int it = 0; it < maxNormIter; it++) {
                auto& ia = iterAccums[it];
                if (ia.totalCount == 0) continue;
                IterRow ir;
                ir.config = cfg.desc;
                ir.tight = mult;
                ir.iteration = it + 1;
                ir.feasibleRate = (double)ia.feasibleCount / ia.totalCount;
                ir.ubCostAvg = avg(ia.ubCosts);
                ir.optCostAvg = avg(ia.optCosts);
                ir.costRatioAvg = (ir.optCostAvg > 0 && !ia.ubCosts.empty()) ?
                                  ir.ubCostAvg / ir.optCostAvg : -1;
                ir.cumTimeMsAvg = avg(ia.cumTimes);
                ir.ercaExpAvg = avg(ia.ercaExps);
                ir.ercaTimeAvg = avg(ia.ercaTimes);
                ir.totalSpeedupAvg = avg(ia.totalSpeedups);
                iterRows.push_back(ir);
            }

            // Progress
            std::cout << "  T=" << mult << "x: " << agg.nSeeds << " seeds, "
                      << "speedup=" << std::fixed << std::setprecision(2) << agg.speedup << "x, "
                      << "nodeRedu=" << std::setprecision(1) << (agg.nodeRedu * 100) << "%, "
                      << "costRatio=" << std::setprecision(3) << agg.costRatio
                      << std::endl;
        }
    }

    csvMain.close();
    csvIter.close();

    // ============================================================
    // TABLE 1: Aggregate Speedup Summary
    // ============================================================
    std::cout << "\n================================================================" << std::endl;
    std::cout << "  TABLE 1: Average Speedup (across " << seeds.size() << " random seeds)" << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << std::setw(13) << "Config"
              << std::setw(6) << "Tight"
              << std::setw(7) << "Seeds"
              << std::setw(10) << "BaseMs"
              << std::setw(10) << "FullMs"
              << std::setw(8) << "NormMs"
              << std::setw(9) << "Speedup"
              << std::setw(10) << "NodeRed%"
              << std::setw(9) << "CostRat"
              << std::endl;
    std::cout << std::string(82, '-') << std::endl;

    for (auto& r : aggRows) {
        std::cout << std::setw(13) << r.config
                  << std::setw(6) << std::fixed << std::setprecision(1) << r.tight
                  << std::setw(7) << r.nSeeds
                  << std::setw(10) << std::setprecision(2) << r.baseTimeAvg
                  << std::setw(10) << r.fullTimeAvg
                  << std::setw(8) << std::setprecision(1) << r.normTimeAvg
                  << std::setw(9) << std::setprecision(2) << r.speedup
                  << std::setw(10) << std::setprecision(1) << (r.nodeRedu * 100)
                  << std::setw(9) << std::setprecision(3) << r.costRatio
                  << std::endl;
    }

    // ============================================================
    // TABLE 2: Time Breakdown (preprocessing vs search)
    // ============================================================
    std::cout << "\n================================================================" << std::endl;
    std::cout << "  TABLE 2: Time Breakdown (Norm preprocessing vs ERCA search)" << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << std::setw(13) << "Config"
              << std::setw(6) << "Tight"
              << std::setw(10) << "BaseMs"
              << std::setw(10) << "NormMs"
              << std::setw(10) << "ERCAMs"
              << std::setw(10) << "TotalMs"
              << std::setw(8) << "Norm%"
              << std::setw(9) << "TimeDiff"
              << std::endl;
    std::cout << std::string(76, '-') << std::endl;

    for (auto& r : aggRows) {
        double ercaMs = r.fullTimeAvg - r.normTimeAvg;
        double normPct = (r.fullTimeAvg > 0) ? (r.normTimeAvg / r.fullTimeAvg * 100) : 0;
        double timeDiff = r.baseTimeAvg - r.fullTimeAvg;
        std::cout << std::setw(13) << r.config
                  << std::setw(6) << std::fixed << std::setprecision(1) << r.tight
                  << std::setw(10) << std::setprecision(2) << r.baseTimeAvg
                  << std::setw(10) << r.normTimeAvg
                  << std::setw(10) << ercaMs
                  << std::setw(10) << r.fullTimeAvg
                  << std::setw(8) << std::setprecision(1) << normPct
                  << std::setw(9) << std::setprecision(2) << timeDiff
                  << std::endl;
    }

    // ============================================================
    // TABLE 3: Per-Iteration UB Quality & Acceleration
    // ============================================================
    std::cout << "\n================================================================" << std::endl;
    std::cout << "  TABLE 3: Per-Iteration Approximate Solution Analysis" << std::endl;
    std::cout << "  (Cost ratio = UB/optimal, >1 means suboptimal)" << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << std::setw(13) << "Config"
              << std::setw(6) << "Tight"
              << std::setw(5) << "Iter"
              << std::setw(8) << "Feas%"
              << std::setw(9) << "CostRat"
              << std::setw(9) << "CumNorm"
              << std::setw(9) << "ERCAExp"
              << std::setw(9) << "Speedup"
              << std::endl;
    std::cout << std::string(68, '-') << std::endl;

    for (auto& r : iterRows) {
        std::cout << std::setw(13) << r.config
                  << std::setw(6) << std::fixed << std::setprecision(1) << r.tight
                  << std::setw(5) << r.iteration
                  << std::setw(8) << std::setprecision(0) << (r.feasibleRate * 100)
                  << std::setw(9) << std::setprecision(3) << r.costRatioAvg
                  << std::setw(9) << std::setprecision(1) << r.cumTimeMsAvg
                  << std::setw(9) << std::setprecision(0) << r.ercaExpAvg
                  << std::setw(9) << std::setprecision(2) << r.totalSpeedupAvg
                  << std::endl;
    }

    // ============================================================
    // TABLE 4: Solution Accuracy Summary by Tightness
    // ============================================================
    std::cout << "\n================================================================" << std::endl;
    std::cout << "  TABLE 4: Solution Accuracy by Tightness (all configs averaged)" << std::endl;
    std::cout << "================================================================" << std::endl;

    for (int ti = 0; ti < numTightness; ti++) {
        double mult = tightness[ti];
        std::cout << "\n  --- Tightness = " << std::fixed << std::setprecision(1) << mult << "x ---" << std::endl;
        std::cout << std::setw(5) << "Iter"
                  << std::setw(10) << "Feas%"
                  << std::setw(12) << "CostRatio"
                  << std::setw(12) << "AvgSpeedup"
                  << std::setw(12) << "AvgExpand"
                  << std::endl;
        std::cout << std::string(51, '-') << std::endl;

        for (int it = 1; it <= maxNormIter; it++) {
            // Aggregate across all configs at this tightness + iteration
            std::vector<double> frates, crats, spds, exps;
            for (auto& r : iterRows) {
                if (std::abs(r.tight - mult) < 0.01 && r.iteration == it) {
                    frates.push_back(r.feasibleRate);
                    if (r.costRatioAvg > 0) crats.push_back(r.costRatioAvg);
                    if (r.totalSpeedupAvg > 0) spds.push_back(r.totalSpeedupAvg);
                    if (r.ercaExpAvg >= 0) exps.push_back(r.ercaExpAvg);
                }
            }
            auto avg = [](const std::vector<double>& v) {
                return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / v.size();
            };
            std::cout << std::setw(5) << it
                      << std::setw(10) << std::setprecision(1) << (avg(frates) * 100)
                      << std::setw(12) << std::setprecision(4) << avg(crats)
                      << std::setw(12) << std::setprecision(2) << avg(spds)
                      << std::setw(12) << std::setprecision(0) << avg(exps)
                      << std::endl;
        }
    }

    // ============================================================
    // TABLE 5: Scalability Analysis (nodes vs speedup)
    // ============================================================
    std::cout << "\n================================================================" << std::endl;
    std::cout << "  TABLE 5: Scalability (2res, extraEdge=0.3, T=2.0x)" << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << std::setw(10) << "Nodes"
              << std::setw(10) << "Edges"
              << std::setw(10) << "BaseMs"
              << std::setw(10) << "FullMs"
              << std::setw(9) << "Speedup"
              << std::setw(10) << "BaseExp"
              << std::setw(10) << "FullExp"
              << std::setw(9) << "NodeRed%"
              << std::endl;
    std::cout << std::string(78, '-') << std::endl;

    for (auto& r : aggRows) {
        // Filter: 2res, normal density, T=2.0x
        if (r.config.find("/2r") != std::string::npos &&
            r.config.find("/d") == std::string::npos &&
            std::abs(r.tight - 2.0) < 0.01) {
            std::cout << std::setw(10) << r.nNodes
                      << std::setw(10) << r.nEdges
                      << std::setw(10) << std::setprecision(2) << r.baseTimeAvg
                      << std::setw(10) << r.fullTimeAvg
                      << std::setw(9) << std::setprecision(2) << r.speedup
                      << std::setw(10) << std::setprecision(0) << r.baseExpAvg
                      << std::setw(10) << r.fullExpAvg
                      << std::setw(9) << std::setprecision(1) << (r.nodeRedu * 100)
                      << std::endl;
        }
    }

    std::cout << "\n\nResults saved to expanded_benchmark_main.csv and expanded_benchmark_iter.csv" << std::endl;

    return 0;
}
