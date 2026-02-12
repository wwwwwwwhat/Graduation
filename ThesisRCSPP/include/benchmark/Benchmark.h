#ifndef THESIS_RCSPP_BENCHMARK_H
#define THESIS_RCSPP_BENCHMARK_H

#include "graph/Graph.h"
#include "graph/Roadmap.h"
#include <string>
#include <vector>

namespace rcspp {

struct BenchmarkEntry {
    std::string method;          // "ERCA", "ERCA+Pulse", "Dijkstra"
    long source;
    long target;
    int numResources;
    long numNodes;
    long numEdges;

    // Results
    bool solved = false;
    bool timeout = false;
    double rtSearch = 0.0;       // search time (seconds)
    double rtTotal = 0.0;        // total time including preprocessing
    long nGenerated = 0;
    long nExpanded = 0;
    long pathLength = 0;         // number of nodes in path
    long primaryCost = 0;        // primary cost of solution
    std::string pathStr;         // path as string
};

class Benchmark {
public:
    Benchmark();
    ~Benchmark();

    // Run ERCA* without pre-processing
    BenchmarkEntry RunERCA(Roadmap& graph, long source, long target,
                           const std::vector<long>& resLimits, double timeLimit);

    // Run ERCA* with Pulse+Normalization pre-processing
    BenchmarkEntry RunERCAWithPulse(Roadmap& graph, long source, long target,
                                     const std::vector<long>& resLimits, double timeLimit);

    // Run simple Dijkstra (no resource constraints, baseline)
    BenchmarkEntry RunDijkstraBaseline(Roadmap& graph, long source, long target);

    // Export results to CSV
    bool ExportCSV(const std::string& filename) const;

    // Add a custom entry
    void AddEntry(const BenchmarkEntry& entry);

    // Get all entries
    const std::vector<BenchmarkEntry>& GetEntries() const;

    // Clear results
    void Clear();

private:
    std::vector<BenchmarkEntry> entries_;
};

} // namespace rcspp

#endif // THESIS_RCSPP_BENCHMARK_H
