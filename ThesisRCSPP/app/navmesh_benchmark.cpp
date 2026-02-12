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
#include <chrono>
#include <iomanip>

using namespace rcspp;

struct TestResult {
    std::string dataset;
    long numNodes;
    long numEdges;
    int numResources;
    long source;
    long target;
    std::string resLimitStr;

    // Dijkstra
    bool dijkSolved = false;
    double dijkTime = 0;
    long dijkCost = 0;

    // ERCA*
    bool ercaSolved = false;
    double ercaTime = 0;
    long ercaCost = 0;
    long ercaExpanded = 0;
    long ercaGenerated = 0;
    std::string ercaPath;

    // ERCA* + Pulse
    bool pulseSolved = false;
    double pulseTime = 0;
    long pulseCost = 0;
    long pulseExpanded = 0;
    long pulseGenerated = 0;
    double normTime = 0;
    bool normFeasible = false;
    int normIter = 0;
    std::string pulsePath;

    double nodeReduction = 0;
};

std::string pathToStr(const std::vector<long>& path) {
    std::ostringstream oss;
    for (size_t i = 0; i < path.size(); i++) {
        if (i > 0) oss << "->";
        oss << path[i];
    }
    return oss.str();
}

TestResult runTest(Roadmap& graph, const std::string& dataset,
                   long source, long target,
                   const std::vector<long>& resLimits, double timeLimit) {
    TestResult r;
    r.dataset = dataset;
    r.numNodes = graph.GetNumberOfNodes();
    r.numEdges = graph.GetNumberOfEdges();
    r.numResources = static_cast<int>(resLimits.size());
    r.source = source;
    r.target = target;

    std::ostringstream rlss;
    rlss << "[";
    for (size_t i = 0; i < resLimits.size(); i++) {
        if (i > 0) rlss << ",";
        rlss << resLimits[i];
    }
    rlss << "]";
    r.resLimitStr = rlss.str();

    std::cout << "  [Dijkstra] ..." << std::flush;
    {
        DijkstraScan dijk;
        dijk.SetGraphPtr(&graph);
        auto t0 = std::chrono::steady_clock::now();
        dijk.Search(target, 0);
        auto t1 = std::chrono::steady_clock::now();
        r.dijkTime = std::chrono::duration<double>(t1 - t0).count();
        long cost = dijk.GetCost(source);
        if (cost >= 0) {
            r.dijkSolved = true;
            r.dijkCost = cost;
        }
    }
    std::cout << " " << r.dijkTime << "s" << std::endl;

    std::cout << "  [ERCA*] ..." << std::flush;
    {
        ERCA erca;
        erca.SetGraph(&graph);
        erca.SetResourceLimits(resLimits);
        auto t0 = std::chrono::steady_clock::now();
        erca.Search(source, target, timeLimit);
        auto t1 = std::chrono::steady_clock::now();
        SearchResult res = erca.GetResult();
        r.ercaTime = std::chrono::duration<double>(t1 - t0).count();
        r.ercaExpanded = res.nExpanded;
        r.ercaGenerated = res.nGenerated;
        if (!res.paths.empty()) {
            r.ercaSolved = true;
            r.ercaCost = res.costs.begin()->second[0];
            r.ercaPath = pathToStr(res.paths.begin()->second);
        }
    }
    std::cout << " " << r.ercaTime << "s, exp=" << r.ercaExpanded << std::endl;

    std::cout << "  [ERCA*+Pulse] ..." << std::flush;
    {
        auto t0 = std::chrono::steady_clock::now();

        // Phase 1: Normalizer + Pulse
        Normalizer norm;
        norm.SetGraph(&graph);
        norm.SetResourceLimits(resLimits);
        NormResult normRes = norm.ComputeUpperBound(source, target, 5);
        r.normTime = normRes.rtTotal;
        r.normFeasible = normRes.feasible;
        r.normIter = normRes.iterations;

        // Phase 2: ERCA* with upper bound
        ERCA erca;
        erca.SetGraph(&graph);
        erca.SetResourceLimits(resLimits);
        if (normRes.feasible) {
            erca.SetUpperBound(normRes.upperBound);
        }
        erca.Search(source, target, timeLimit);

        auto t1 = std::chrono::steady_clock::now();
        SearchResult res = erca.GetResult();
        r.pulseTime = std::chrono::duration<double>(t1 - t0).count();
        r.pulseExpanded = res.nExpanded;
        r.pulseGenerated = res.nGenerated;
        if (!res.paths.empty()) {
            r.pulseSolved = true;
            r.pulseCost = res.costs.begin()->second[0];
            r.pulsePath = pathToStr(res.paths.begin()->second);
        }
    }
    std::cout << " " << r.pulseTime << "s, exp=" << r.pulseExpanded << std::endl;

    // Node reduction
    if (r.ercaExpanded > 0 && r.pulseExpanded > 0) {
        r.nodeReduction = 1.0 - (double)r.pulseExpanded / r.ercaExpanded;
    }

    return r;
}

int main() {
    std::cout << "=== NavMesh ERCA* Comprehensive Benchmark ===" << std::endl;
    std::cout << std::endl;

    std::string navmeshDir;
    {
        std::vector<std::string> tryPaths = {
            "../../NavMeshERCA_Experiment/build/Release/",
            "../../../NavMeshERCA_Experiment/build/Release/",
            "C:/Users/DELL/Graduation/NavMeshERCA_Experiment/build/Release/",
        };
        for (auto& p : tryPaths) {
            std::ifstream test(p + "navmesh_small_cost.gr");
            if (test.good()) { navmeshDir = p; break; }
        }
        if (navmeshDir.empty()) {
            std::cerr << "Cannot find NavMesh DIMACS files!" << std::endl;
            return 1;
        }
        std::cout << "Data dir: " << navmeshDir << std::endl;
    }

    struct DatasetConfig {
        std::string name;
        std::string costFile;
        std::vector<std::string> resFiles;
    };

    std::vector<DatasetConfig> datasets = {
        {"small(69)",  navmeshDir + "navmesh_small_cost.gr",
         {navmeshDir + "navmesh_small_res1.gr", navmeshDir + "navmesh_small_res2.gr"}},
        {"medium(183)", navmeshDir + "navmesh_medium_cost.gr",
         {navmeshDir + "navmesh_medium_res1.gr", navmeshDir + "navmesh_medium_res2.gr"}},
        {"large(447)",  navmeshDir + "navmesh_large_cost.gr",
         {navmeshDir + "navmesh_large_res1.gr", navmeshDir + "navmesh_large_res2.gr"}},
        {"xlarge(708)", navmeshDir + "navmesh_xlarge_cost.gr",
         {navmeshDir + "navmesh_xlarge_res1.gr", navmeshDir + "navmesh_xlarge_res2.gr"}},
    };

    // Resource tightness levels: multiplier of Dijkstra shortest resource path
    double tightness[] = {1.5, 2.0, 3.0, 5.0};
    int numTightness = 4;

    std::vector<TestResult> allResults;

    for (auto& ds : datasets) {
        std::cout << "=== Dataset: " << ds.name << " ===" << std::endl;

        Roadmap graph;
        try {
            graph = Roadmap::LoadDIMACS(ds.costFile, ds.resFiles);
        } catch (const std::exception& e) {
            std::cout << "  Skip (load failed): " << e.what() << std::endl;
            continue;
        }

        long nNodes = graph.GetNumberOfNodes();
        long nEdges = graph.GetNumberOfEdges();
        std::cout << "  Nodes=" << nNodes << ", Edges=" << nEdges << std::endl;

        if (nNodes == 0) continue;

        // Find connected source-target pair with max distance
        DijkstraScan primaryDijk;
        primaryDijk.SetGraphPtr(&graph);
        long firstNode = *graph.GetNodes().begin();
        primaryDijk.Search(firstNode, 0);

        // Find the farthest reachable node as source
        long source = firstNode, target = firstNode;
        long maxDist = 0;
        for (auto v : graph.GetNodes()) {
            long d = primaryDijk.GetCost(v);
            if (d > maxDist) {
                maxDist = d;
                source = v;
            }
        }
        // Now find farthest from source
        DijkstraScan srcDijk;
        srcDijk.SetGraphPtr(&graph);
        srcDijk.Search(source, 0);
        maxDist = 0;
        for (auto v : graph.GetNodes()) {
            long d = srcDijk.GetCost(v);
            if (d > maxDist) {
                maxDist = d;
                target = v;
            }
        }
        std::cout << "  Source=" << source << ", Target=" << target << ", dist=" << maxDist << std::endl;

        if (maxDist <= 0) {
            std::cout << "  Skip (no reachable target)" << std::endl;
            continue;
        }

        // Get Dijkstra min resource costs for setting limits
        std::vector<long> minResCosts(2, 0);
        for (int r = 0; r < 2; r++) {
            DijkstraScan dijk;
            dijk.SetGraphPtr(&graph);
            dijk.Search(target, r + 1);
            long cost = dijk.GetCost(source);
            if (cost <= 0) {
                std::cout << "  Skip (resource dim " << r << " unreachable)" << std::endl;
                continue;
            }
            minResCosts[r] = cost;
        }
        std::cout << "  Min resource costs: [" << minResCosts[0] << "," << minResCosts[1] << "]" << std::endl;

        if (minResCosts[0] <= 0 || minResCosts[1] <= 0) continue;

        // Test with different tightness levels
        for (int t = 0; t < numTightness; t++) {
            double mult = tightness[t];
            std::vector<long> resLimits;
            for (int r = 0; r < 2; r++) {
                resLimits.push_back(static_cast<long>(minResCosts[r] * mult));
            }

            std::cout << "  --- Tightness=" << mult << "x, limits=["
                      << resLimits[0] << "," << resLimits[1] << "] ---" << std::endl;

            TestResult result = runTest(graph, ds.name, source, target, resLimits, 30.0);
            allResults.push_back(result);
        }
        std::cout << std::endl;
    }

    // === Output results table ===
    std::cout << "\n=== RESULTS SUMMARY ===" << std::endl;
    std::cout << std::setw(15) << "Dataset"
              << std::setw(6) << "N"
              << std::setw(6) << "E"
              << std::setw(10) << "Limits"
              << std::setw(10) << "Dijk(ms)"
              << std::setw(10) << "ERCA(ms)"
              << std::setw(8) << "ERCAex"
              << std::setw(12) << "ERCA+P(ms)"
              << std::setw(8) << "E+Pex"
              << std::setw(10) << "Reduct%"
              << std::setw(8) << "NormOK"
              << std::endl;

    for (auto& r : allResults) {
        std::cout << std::setw(15) << r.dataset
                  << std::setw(6) << r.numNodes
                  << std::setw(6) << r.numEdges
                  << std::setw(10) << r.resLimitStr
                  << std::setw(10) << std::fixed << std::setprecision(3) << r.dijkTime * 1000
                  << std::setw(10) << r.ercaTime * 1000
                  << std::setw(8) << r.ercaExpanded
                  << std::setw(12) << r.pulseTime * 1000
                  << std::setw(8) << r.pulseExpanded
                  << std::setw(10) << std::setprecision(1) << r.nodeReduction * 100
                  << std::setw(8) << (r.normFeasible ? "Y" : "N")
                  << std::endl;
    }

    // === Output detailed results to file ===
    std::ofstream fout("navmesh_benchmark_results.txt");
    if (fout.is_open()) {
        fout << "dataset,nodes,edges,resLimits,dijkTime_ms,dijkCost,ercaTime_ms,ercaCost,ercaExpanded,ercaGenerated,"
             << "pulseTime_ms,pulseCost,pulseExpanded,pulseGenerated,normTime_ms,normFeasible,normIter,nodeReduction,ercaPath,pulsePath" << std::endl;
        for (auto& r : allResults) {
            fout << r.dataset << ","
                 << r.numNodes << ","
                 << r.numEdges << ","
                 << r.resLimitStr << ","
                 << r.dijkTime * 1000 << ","
                 << r.dijkCost << ","
                 << r.ercaTime * 1000 << ","
                 << r.ercaCost << ","
                 << r.ercaExpanded << ","
                 << r.ercaGenerated << ","
                 << r.pulseTime * 1000 << ","
                 << r.pulseCost << ","
                 << r.pulseExpanded << ","
                 << r.pulseGenerated << ","
                 << r.normTime * 1000 << ","
                 << (r.normFeasible ? 1 : 0) << ","
                 << r.normIter << ","
                 << r.nodeReduction << ","
                 << "\"" << r.ercaPath << "\","
                 << "\"" << r.pulsePath << "\"" << std::endl;
        }
        fout.close();
        std::cout << "\nDetailed results saved to navmesh_benchmark_results.txt" << std::endl;
    }

    return 0;
}
