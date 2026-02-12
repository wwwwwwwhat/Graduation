#include "benchmark/Benchmark.h"
#include "algorithm/ERCA.h"
#include "algorithm/Pulse.h"
#include "algorithm/Normalizer.h"
#include "algorithm/Dijkstra.h"
#include <fstream>
#include <chrono>
#include <sstream>

namespace rcspp {

Benchmark::Benchmark() {}
Benchmark::~Benchmark() {}

BenchmarkEntry Benchmark::RunERCA(Roadmap& graph, long source, long target,
                                   const std::vector<long>& resLimits, double timeLimit) {
    BenchmarkEntry entry;
    entry.method = "ERCA";
    entry.source = source;
    entry.target = target;
    entry.numResources = static_cast<int>(resLimits.size());
    entry.numNodes = graph.GetNumberOfNodes();
    entry.numEdges = graph.GetNumberOfEdges();

    auto tstart = std::chrono::steady_clock::now();

    ERCA erca;
    erca.SetGraph(&graph);
    erca.SetResourceLimits(resLimits);
    erca.Search(source, target, timeLimit);

    auto tend = std::chrono::steady_clock::now();

    SearchResult res = erca.GetResult();
    entry.timeout = res.timeout;
    entry.rtSearch = res.rtSearch;
    entry.rtTotal = std::chrono::duration<double>(tend - tstart).count();
    entry.nGenerated = res.nGenerated;
    entry.nExpanded = res.nExpanded;

    if (!res.paths.empty()) {
        entry.solved = true;
        auto& firstPath = res.paths.begin()->second;
        auto& firstCost = res.costs.begin()->second;
        entry.pathLength = static_cast<long>(firstPath.size());
        entry.primaryCost = firstCost[0];

        std::ostringstream oss;
        for (size_t i = 0; i < firstPath.size(); i++) {
            if (i > 0) oss << "->";
            oss << firstPath[i];
        }
        entry.pathStr = oss.str();
    }

    entries_.push_back(entry);
    return entry;
}

BenchmarkEntry Benchmark::RunERCAWithPulse(Roadmap& graph, long source, long target,
                                             const std::vector<long>& resLimits, double timeLimit) {
    BenchmarkEntry entry;
    entry.method = "ERCA+Pulse";
    entry.source = source;
    entry.target = target;
    entry.numResources = static_cast<int>(resLimits.size());
    entry.numNodes = graph.GetNumberOfNodes();
    entry.numEdges = graph.GetNumberOfEdges();

    auto tstart = std::chrono::steady_clock::now();

    // Phase 1: Normalization + Pulse for upper bound
    Normalizer norm;
    norm.SetGraph(&graph);
    norm.SetResourceLimits(resLimits);
    NormResult normRes = norm.ComputeUpperBound(source, target, 5);

    // Phase 2: ERCA* with upper bound
    ERCA erca;
    erca.SetGraph(&graph);
    erca.SetResourceLimits(resLimits);
    if (normRes.feasible) {
        erca.SetUpperBound(normRes.upperBound);
    }
    erca.Search(source, target, timeLimit);

    auto tend = std::chrono::steady_clock::now();

    SearchResult res = erca.GetResult();
    entry.timeout = res.timeout;
    entry.rtSearch = res.rtSearch;
    entry.rtTotal = std::chrono::duration<double>(tend - tstart).count();
    entry.nGenerated = res.nGenerated;
    entry.nExpanded = res.nExpanded;

    if (!res.paths.empty()) {
        entry.solved = true;
        auto& firstPath = res.paths.begin()->second;
        auto& firstCost = res.costs.begin()->second;
        entry.pathLength = static_cast<long>(firstPath.size());
        entry.primaryCost = firstCost[0];

        std::ostringstream oss;
        for (size_t i = 0; i < firstPath.size(); i++) {
            if (i > 0) oss << "->";
            oss << firstPath[i];
        }
        entry.pathStr = oss.str();
    }

    entries_.push_back(entry);
    return entry;
}

BenchmarkEntry Benchmark::RunDijkstraBaseline(Roadmap& graph, long source, long target) {
    BenchmarkEntry entry;
    entry.method = "Dijkstra";
    entry.source = source;
    entry.target = target;
    entry.numResources = 0;
    entry.numNodes = graph.GetNumberOfNodes();
    entry.numEdges = graph.GetNumberOfEdges();

    auto tstart = std::chrono::steady_clock::now();

    DijkstraScan dijkstra;
    dijkstra.SetGraphPtr(&graph);
    dijkstra.Search(target, 0); // dim 0 = primary cost

    auto tend = std::chrono::steady_clock::now();

    entry.rtSearch = std::chrono::duration<double>(tend - tstart).count();
    entry.rtTotal = entry.rtSearch;

    long cost = dijkstra.GetCost(source);
    if (cost >= 0) {
        entry.solved = true;
        entry.primaryCost = cost;
        auto path = dijkstra.GetPath(source);
        entry.pathLength = static_cast<long>(path.size());

        std::ostringstream oss;
        for (size_t i = 0; i < path.size(); i++) {
            if (i > 0) oss << "->";
            oss << path[i];
        }
        entry.pathStr = oss.str();
    }

    // Count all expanded nodes as nExpanded (Dijkstra expands all reachable)
    entry.nExpanded = graph.GetNumberOfNodes();
    entry.nGenerated = graph.GetNumberOfNodes();

    entries_.push_back(entry);
    return entry;
}

bool Benchmark::ExportCSV(const std::string& filename) const {
    std::ofstream fout(filename);
    if (!fout.is_open()) return false;

    fout << "method,source,target,numResources,numNodes,numEdges,"
         << "solved,timeout,rtSearch,rtTotal,nGenerated,nExpanded,"
         << "pathLength,primaryCost,path\n";

    for (auto& e : entries_) {
        fout << e.method << ","
             << e.source << ","
             << e.target << ","
             << e.numResources << ","
             << e.numNodes << ","
             << e.numEdges << ","
             << (e.solved ? 1 : 0) << ","
             << (e.timeout ? 1 : 0) << ","
             << e.rtSearch << ","
             << e.rtTotal << ","
             << e.nGenerated << ","
             << e.nExpanded << ","
             << e.pathLength << ","
             << e.primaryCost << ","
             << "\"" << e.pathStr << "\"\n";
    }

    return true;
}

void Benchmark::AddEntry(const BenchmarkEntry& entry) {
    entries_.push_back(entry);
}

const std::vector<BenchmarkEntry>& Benchmark::GetEntries() const {
    return entries_;
}

void Benchmark::Clear() {
    entries_.clear();
}

} // namespace rcspp
