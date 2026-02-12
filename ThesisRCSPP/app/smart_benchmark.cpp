#include "graph/Roadmap.h"
#include "algorithm/ERCA.h"
#include "algorithm/Normalizer.h"
#include "algorithm/Dijkstra.h"
#include <iostream>
#include <fstream>
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
// Graph generation
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
// Compute resource usage along a path on the original graph
// ============================================================
std::vector<long> computePathResources(Graph& graph, const std::vector<long>& path, int numRes) {
    std::vector<long> usage(numRes, 0);
    for (size_t i = 0; i + 1 < path.size(); i++) {
        CostVector c = graph.GetCost(path[i], path[i + 1]);
        for (int r = 0; r < numRes; r++) {
            usage[r] += c[r + 1];
        }
    }
    return usage;
}

// ============================================================
// Smart limit computation
// LB[r] = min resource cost (Dijkstra on dim r+1, reverse from target)
// UB[r] = resource cost of the cost-optimal path (Dijkstra on dim 0)
// Limit[r] = LB[r] + alpha * (UB[r] - LB[r])
// ============================================================
struct SmartLimitInfo {
    std::vector<long> lb;       // per-resource lower bound
    std::vector<long> ub;       // per-resource upper bound (cost-optimal path)
    std::vector<long> limits;   // computed limits for given alpha
    long optPathCost;           // primary cost of cost-optimal path
    bool valid;
};

SmartLimitInfo computeSmartLimits(Roadmap& graph, long source, long target,
                                   int numRes, double alpha) {
    SmartLimitInfo info;
    info.valid = false;
    info.lb.resize(numRes);
    info.ub.resize(numRes);
    info.limits.resize(numRes);

    // Step 1: LB = reverse Dijkstra per resource dimension
    for (int r = 0; r < numRes; r++) {
        DijkstraScan d;
        d.SetGraphPtr(&graph);
        d.Search(target, r + 1);
        long c = d.GetCost(source);
        if (c <= 0) return info; // unreachable
        info.lb[r] = c;
    }

    // Step 2: UB = cost-optimal path's resource usage
    // Run reverse Dijkstra on dim 0 (primary cost) from target
    DijkstraScan dCost;
    dCost.SetGraphPtr(&graph);
    dCost.Search(target, 0);
    info.optPathCost = dCost.GetCost(source);
    if (info.optPathCost <= 0) return info;

    // Get the cost-optimal path (source -> ... -> target)
    std::vector<long> costOptPath = dCost.GetPath(source);
    if (costOptPath.empty()) return info;

    // Compute resource usage along this path
    info.ub = computePathResources(graph, costOptPath, numRes);

    // Step 3: Compute limits
    for (int r = 0; r < numRes; r++) {
        // Ensure UB >= LB (it should always be, but safety check)
        if (info.ub[r] < info.lb[r]) info.ub[r] = info.lb[r];
        info.limits[r] = info.lb[r] + static_cast<long>(alpha * (info.ub[r] - info.lb[r]));
        // Ensure limit >= LB
        if (info.limits[r] < info.lb[r]) info.limits[r] = info.lb[r];
    }

    info.valid = true;
    return info;
}

// ============================================================
// Run ERCA* with given parameters
// ============================================================
struct BenchResult {
    double totalTimeMs = 0;
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
    std::cout << "  Smart Benchmark: alpha difficulty coefficient" << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << std::endl;
    std::cout << "alpha=0.0: Limit=LB (tightest, like t=1.0)" << std::endl;
    std::cout << "alpha=1.0: Limit=UB (cost-optimal path just feasible)" << std::endl;
    std::cout << "alpha=0.5: Sweet spot for testing" << std::endl;
    std::cout << std::endl;

    struct TestConfig {
        int gridSize;
        int numRes;
        double extraEdge;
        std::string desc;
    };

    std::vector<TestConfig> configs = {
        {15, 2, 0.3, "15x15/2r"},
        {20, 2, 0.3, "20x20/2r"},
        {25, 2, 0.3, "25x25/2r"},
        {30, 2, 0.3, "30x30/2r"},
        {35, 2, 0.3, "35x35/2r"},
        {20, 2, 0.6, "20x20/2r/d"},
        {25, 2, 0.6, "25x25/2r/d"},
        {30, 2, 0.6, "30x30/2r/d"},
        {20, 3, 0.3, "20x20/3r"},
        {25, 3, 0.3, "25x25/3r"},
    };

    std::vector<unsigned> seeds = {42, 123, 7, 2024, 999};
    double alphas[] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    int numAlphas = 11;

    // CSV output
    std::ofstream csvOut("smart_benchmark.csv");
    csvOut << "config,alpha,seed,nodes,edges,"
           << "LB_r0,UB_r0,Limit_r0,";
    csvOut << "base_time_ms,base_exp,base_cost,base_solved,"
           << "full_time_ms,full_exp,full_cost,full_solved,"
           << "norm_time_ms,norm_ok,"
           << "infl_time_ms,infl_exp,infl_cost,infl_solved,"
           << "speedup_full,speedup_infl,node_red_full,node_red_infl" << std::endl;

    // Aggregate per (config, alpha)
    struct AggRow {
        std::string config;
        double alpha;
        int nSeeds = 0;
        // averages
        double lbAvg = 0, ubAvg = 0;
        double baseTimeAvg = 0, baseExpAvg = 0, baseCostAvg = 0;
        double fullTimeAvg = 0, fullExpAvg = 0, fullCostAvg = 0;
        double inflTimeAvg = 0, inflExpAvg = 0, inflCostAvg = 0;
        double normTimeAvg = 0;
        double normSuccessRate = 0;
        double baseSolveRate = 0;
        double fullSolveRate = 0;
        double inflSolveRate = 0;
    };
    std::vector<AggRow> aggRows;

    for (auto& cfg : configs) {
        std::cout << "\n=== " << cfg.desc << " ===" << std::endl;

        for (int ai = 0; ai < numAlphas; ai++) {
            double alpha = alphas[ai];

            std::vector<double> lbs, ubs;
            std::vector<double> baseTimes, baseExps, baseCosts;
            std::vector<double> fullTimes, fullExps, fullCosts;
            std::vector<double> inflTimes, inflExps, inflCosts;
            std::vector<double> normTimes;
            int normOKcount = 0;
            int baseSolved = 0, fullSolved = 0, inflSolved = 0;
            int validSeeds = 0;

            for (auto seed : seeds) {
                Roadmap graph = generateGraph(cfg.gridSize, cfg.numRes, cfg.extraEdge, seed);
                long N = graph.GetNumberOfNodes();
                long E = graph.GetNumberOfEdges();
                long source = 1, target = N;

                SmartLimitInfo sli = computeSmartLimits(graph, source, target, cfg.numRes, alpha);
                if (!sli.valid) continue;

                validSeeds++;
                lbs.push_back(sli.lb[0]);
                ubs.push_back(sli.ub[0]);

                // (A) Baseline: exact ERCA* (w=1.0)
                BenchResult base = runERCA(graph, source, target, sli.limits, 1.0, nullptr, 60.0);

                // (B) Full strategy: Norm + UB + w=1.5
                auto normStart = std::chrono::steady_clock::now();
                Normalizer norm;
                norm.SetGraph(&graph);
                norm.SetResourceLimits(sli.limits);
                NormResult normRes = norm.ComputeUpperBoundERCA(source, target, 5);
                auto normEnd = std::chrono::steady_clock::now();
                double normMs = std::chrono::duration<double>(normEnd - normStart).count() * 1000;

                BenchResult full;
                if (normRes.feasible) {
                    full = runERCA(graph, source, target, sli.limits, 1.5, &normRes.upperBound, 60.0);
                } else {
                    full = runERCA(graph, source, target, sli.limits, 1.5, nullptr, 60.0);
                }
                full.totalTimeMs += normMs;

                // Use normalization result if ERCA* with inflation found nothing
                long fullCostVal = full.solved ? full.cost : -1;
                bool fullSolvedFinal = full.solved;
                if (!full.solved && normRes.feasible) {
                    fullCostVal = normRes.upperBound[0];
                    fullSolvedFinal = true;
                }

                // (C) Inflation only: w=1.5, no norm
                BenchResult infl = runERCA(graph, source, target, sli.limits, 1.5, nullptr, 60.0);

                if (normRes.feasible) normOKcount++;
                if (base.solved && !base.timeout) baseSolved++;
                if (fullSolvedFinal) fullSolved++;
                if (infl.solved && !infl.timeout) inflSolved++;

                baseTimes.push_back(base.totalTimeMs);
                baseExps.push_back(base.expanded);
                baseCosts.push_back(base.solved ? base.cost : -1);
                fullTimes.push_back(full.totalTimeMs);
                fullExps.push_back(full.expanded);
                fullCosts.push_back(fullCostVal);
                inflTimes.push_back(infl.totalTimeMs);
                inflExps.push_back(infl.expanded);
                inflCosts.push_back(infl.solved ? infl.cost : -1);
                normTimes.push_back(normMs);

                // CSV per-seed
                double spFull = (full.totalTimeMs > 0) ? base.totalTimeMs / full.totalTimeMs : 0;
                double spInfl = (infl.totalTimeMs > 0) ? base.totalTimeMs / infl.totalTimeMs : 0;
                double nrFull = (base.expanded > 0) ? (1.0 - (double)full.expanded / base.expanded) : 0;
                double nrInfl = (base.expanded > 0) ? (1.0 - (double)infl.expanded / base.expanded) : 0;

                csvOut << "\"" << cfg.desc << "\"," << alpha << "," << seed << ","
                       << N << "," << E << ","
                       << sli.lb[0] << "," << sli.ub[0] << "," << sli.limits[0] << ","
                       << base.totalTimeMs << "," << base.expanded << "," << base.cost << "," << (base.solved?1:0) << ","
                       << full.totalTimeMs << "," << full.expanded << "," << fullCostVal << "," << (fullSolvedFinal?1:0) << ","
                       << normMs << "," << (normRes.feasible?1:0) << ","
                       << infl.totalTimeMs << "," << infl.expanded << "," << (infl.solved?infl.cost:-1) << "," << (infl.solved?1:0) << ","
                       << spFull << "," << spInfl << "," << nrFull << "," << nrInfl << std::endl;
            }

            if (validSeeds == 0) continue;

            auto avg = [](const std::vector<double>& v) {
                return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / v.size();
            };

            AggRow agg;
            agg.config = cfg.desc;
            agg.alpha = alpha;
            agg.nSeeds = validSeeds;
            agg.lbAvg = avg(lbs);
            agg.ubAvg = avg(ubs);
            agg.baseTimeAvg = avg(baseTimes);
            agg.baseExpAvg = avg(baseExps);
            agg.baseCostAvg = avg(baseCosts);
            agg.fullTimeAvg = avg(fullTimes);
            agg.fullExpAvg = avg(fullExps);
            agg.fullCostAvg = avg(fullCosts);
            agg.inflTimeAvg = avg(inflTimes);
            agg.inflExpAvg = avg(inflExps);
            agg.inflCostAvg = avg(inflCosts);
            agg.normTimeAvg = avg(normTimes);
            agg.normSuccessRate = (double)normOKcount / validSeeds;
            agg.baseSolveRate = (double)baseSolved / validSeeds;
            agg.fullSolveRate = (double)fullSolved / validSeeds;
            agg.inflSolveRate = (double)inflSolved / validSeeds;
            aggRows.push_back(agg);

            std::cout << "  a=" << std::fixed << std::setprecision(1) << alpha
                      << ": seeds=" << validSeeds
                      << " baseSolve=" << std::setprecision(0) << (agg.baseSolveRate*100) << "%"
                      << " baseExp=" << std::setprecision(0) << agg.baseExpAvg
                      << " fullExp=" << agg.fullExpAvg
                      << " inflExp=" << agg.inflExpAvg
                      << " normOK=" << std::setprecision(0) << (agg.normSuccessRate*100) << "%"
                      << std::endl;
        }
    }

    csvOut.close();

    // ============================================================
    // TABLE 1: LB/UB Range Info
    // ============================================================
    std::cout << "\n================================================================" << std::endl;
    std::cout << "  TABLE 1: Resource Bound Range (resource dim 0)" << std::endl;
    std::cout << "  LB = min resource cost, UB = cost-optimal path resource" << std::endl;
    std::cout << "================================================================" << std::endl;

    // Show one row per config (alpha=0 and alpha=1 limits)
    std::string lastConfig = "";
    for (auto& r : aggRows) {
        if (r.config == lastConfig) continue;
        lastConfig = r.config;
        // Find alpha=0 and alpha=1 for this config
        double lb = 0, ub = 0;
        for (auto& rr : aggRows) {
            if (rr.config == r.config && rr.alpha < 0.01) lb = rr.lbAvg;
            if (rr.config == r.config && rr.alpha > 0.99) ub = rr.ubAvg;
        }
        std::cout << std::setw(13) << r.config
                  << "  LB(avg)=" << std::setw(6) << std::setprecision(0) << lb
                  << "  UB(avg)=" << std::setw(6) << ub
                  << "  ratio(UB/LB)=" << std::setprecision(2) << (lb > 0 ? ub / lb : 0)
                  << std::endl;
    }

    // ============================================================
    // TABLE 2: Main Results - Solve Rate & Expanded Nodes by alpha
    // ============================================================
    std::cout << "\n================================================================" << std::endl;
    std::cout << "  TABLE 2: Solve Rate & Expanded Nodes by alpha" << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << std::setw(13) << "Config"
              << std::setw(6) << "alpha"
              << std::setw(7) << "Solve%"
              << std::setw(9) << "BaseExp"
              << std::setw(9) << "FullExp"
              << std::setw(9) << "InflExp"
              << std::setw(8) << "NormOK%"
              << std::setw(9) << "NodeRed%"
              << std::endl;
    std::cout << std::string(70, '-') << std::endl;

    for (auto& r : aggRows) {
        double nodeRed = (r.baseExpAvg > 0) ? (1.0 - r.fullExpAvg / r.baseExpAvg) * 100 : 0;
        std::cout << std::setw(13) << r.config
                  << std::setw(6) << std::fixed << std::setprecision(1) << r.alpha
                  << std::setw(7) << std::setprecision(0) << (r.baseSolveRate * 100)
                  << std::setw(9) << std::setprecision(0) << r.baseExpAvg
                  << std::setw(9) << r.fullExpAvg
                  << std::setw(9) << r.inflExpAvg
                  << std::setw(8) << std::setprecision(0) << (r.normSuccessRate * 100)
                  << std::setw(9) << std::setprecision(1) << nodeRed
                  << std::endl;
    }

    // ============================================================
    // TABLE 3: Time Comparison by alpha
    // ============================================================
    std::cout << "\n================================================================" << std::endl;
    std::cout << "  TABLE 3: Time Comparison (ms) by alpha" << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << std::setw(13) << "Config"
              << std::setw(6) << "alpha"
              << std::setw(9) << "BaseMs"
              << std::setw(9) << "FullMs"
              << std::setw(9) << "InflMs"
              << std::setw(9) << "NormMs"
              << std::setw(8) << "SpFull"
              << std::setw(8) << "SpInfl"
              << std::endl;
    std::cout << std::string(62, '-') << std::endl;

    for (auto& r : aggRows) {
        double spFull = (r.fullTimeAvg > 0) ? r.baseTimeAvg / r.fullTimeAvg : 0;
        double spInfl = (r.inflTimeAvg > 0) ? r.baseTimeAvg / r.inflTimeAvg : 0;
        std::cout << std::setw(13) << r.config
                  << std::setw(6) << std::fixed << std::setprecision(1) << r.alpha
                  << std::setw(9) << std::setprecision(2) << r.baseTimeAvg
                  << std::setw(9) << r.fullTimeAvg
                  << std::setw(9) << r.inflTimeAvg
                  << std::setw(9) << r.normTimeAvg
                  << std::setw(8) << std::setprecision(2) << spFull
                  << std::setw(8) << spInfl
                  << std::endl;
    }

    // ============================================================
    // TABLE 4: Solution Quality by alpha
    // ============================================================
    std::cout << "\n================================================================" << std::endl;
    std::cout << "  TABLE 4: Solution Quality (cost ratio vs optimal)" << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << std::setw(13) << "Config"
              << std::setw(6) << "alpha"
              << std::setw(9) << "OptCost"
              << std::setw(9) << "FullCost"
              << std::setw(9) << "InflCost"
              << std::setw(9) << "FullRat"
              << std::setw(9) << "InflRat"
              << std::endl;
    std::cout << std::string(55, '-') << std::endl;

    for (auto& r : aggRows) {
        if (r.baseSolveRate < 0.5) continue; // skip if mostly unsolvable
        double fullRat = (r.baseCostAvg > 0 && r.fullCostAvg > 0) ? r.fullCostAvg / r.baseCostAvg : -1;
        double inflRat = (r.baseCostAvg > 0 && r.inflCostAvg > 0) ? r.inflCostAvg / r.baseCostAvg : -1;
        std::cout << std::setw(13) << r.config
                  << std::setw(6) << std::fixed << std::setprecision(1) << r.alpha
                  << std::setw(9) << std::setprecision(0) << r.baseCostAvg
                  << std::setw(9) << r.fullCostAvg
                  << std::setw(9) << r.inflCostAvg
                  << std::setw(9) << std::setprecision(3) << fullRat
                  << std::setw(9) << inflRat
                  << std::endl;
    }

    // ============================================================
    // TABLE 5: alpha Sweet Spot Analysis (all configs aggregated)
    // ============================================================
    std::cout << "\n================================================================" << std::endl;
    std::cout << "  TABLE 5: Alpha Sweet Spot (all configs aggregated)" << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << std::setw(6) << "alpha"
              << std::setw(9) << "Solve%"
              << std::setw(9) << "BaseExp"
              << std::setw(9) << "FullExp"
              << std::setw(9) << "InflExp"
              << std::setw(9) << "NormOK%"
              << std::setw(9) << "SpFull"
              << std::setw(9) << "SpInfl"
              << std::setw(9) << "CostRat"
              << std::endl;
    std::cout << std::string(68, '-') << std::endl;

    for (int ai = 0; ai < numAlphas; ai++) {
        double alpha = alphas[ai];
        std::vector<double> solves, bExps, fExps, iExps, norms, spFs, spIs, cRats;

        for (auto& r : aggRows) {
            if (std::abs(r.alpha - alpha) > 0.01) continue;
            solves.push_back(r.baseSolveRate);
            bExps.push_back(r.baseExpAvg);
            fExps.push_back(r.fullExpAvg);
            iExps.push_back(r.inflExpAvg);
            norms.push_back(r.normSuccessRate);
            if (r.fullTimeAvg > 0 && r.baseSolveRate > 0.5)
                spFs.push_back(r.baseTimeAvg / r.fullTimeAvg);
            if (r.inflTimeAvg > 0 && r.baseSolveRate > 0.5)
                spIs.push_back(r.baseTimeAvg / r.inflTimeAvg);
            if (r.baseCostAvg > 0 && r.fullCostAvg > 0 && r.baseSolveRate > 0.5)
                cRats.push_back(r.fullCostAvg / r.baseCostAvg);
        }

        auto avg = [](const std::vector<double>& v) {
            return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / v.size();
        };

        std::cout << std::setw(6) << std::fixed << std::setprecision(1) << alpha
                  << std::setw(9) << std::setprecision(0) << (avg(solves) * 100)
                  << std::setw(9) << std::setprecision(0) << avg(bExps)
                  << std::setw(9) << avg(fExps)
                  << std::setw(9) << avg(iExps)
                  << std::setw(9) << std::setprecision(0) << (avg(norms) * 100)
                  << std::setw(9) << std::setprecision(2) << avg(spFs)
                  << std::setw(9) << avg(spIs)
                  << std::setw(9) << std::setprecision(3) << avg(cRats)
                  << std::endl;
    }

    std::cout << "\n\nResults saved to smart_benchmark.csv" << std::endl;

    return 0;
}
