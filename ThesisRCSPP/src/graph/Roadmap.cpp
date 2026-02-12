#include "graph/Roadmap.h"
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace rcspp {

Roadmap::Roadmap() {}

void Roadmap::Init(long numNodes, size_t costDim) {
    numNodes_ = numNodes;
    costDim_ = costDim;
    numEdges_ = 0;
    nodes.clear();
    adjlist.clear();
    adjlistRev.clear();
}

bool Roadmap::HasNode(long v) {
    return nodes.count(v) > 0;
}

std::unordered_set<long> Roadmap::GetSuccs(long v) {
    std::unordered_set<long> result;
    if (adjlist.count(v)) {
        for (auto& kv : adjlist[v]) {
            result.insert(kv.first);
        }
    }
    return result;
}

std::unordered_set<long> Roadmap::GetPreds(long v) {
    std::unordered_set<long> result;
    if (adjlistRev.count(v)) {
        for (auto& kv : adjlistRev[v]) {
            result.insert(kv.first);
        }
    }
    return result;
}

CostVector Roadmap::GetCost(long u, long v) {
    if (adjlist.count(u) && adjlist[u].count(v)) {
        return adjlist[u][v];
    }
    return CostVector(std::numeric_limits<long>::max(), costDim_);
}

size_t Roadmap::GetCostDim() {
    return costDim_;
}

std::unordered_set<long> Roadmap::GetNodes() {
    return nodes;
}

long Roadmap::GetNumberOfNodes() {
    return static_cast<long>(nodes.size());
}

long Roadmap::GetNumberOfEdges() {
    return numEdges_;
}

void Roadmap::AddNode(long v) {
    nodes.insert(v);
}

void Roadmap::AddEdge(long u, long v, const CostVector& cost) {
    AddNode(u);
    AddNode(v);
    adjlist[u][v] = cost;
    adjlistRev[v][u] = cost;
    numEdges_++;
}

bool Roadmap::HasEdge(long u, long v) {
    return adjlist.count(u) && adjlist[u].count(v);
}

// Parse a single DIMACS .gr file and return edge list with single-dim costs
static std::vector<std::tuple<long, long, long>> ParseDIMACSFile(
    const std::string& filename, long& outNodes, long& outEdges) {

    std::vector<std::tuple<long, long, long>> edges;
    std::ifstream fin(filename);
    if (!fin.is_open()) {
        throw std::runtime_error("Cannot open file: " + filename);
    }

    std::string line;
    while (std::getline(fin, line)) {
        if (line.empty() || line[0] == 'c') continue;
        if (line[0] == 'p') {
            // p sp <nodes> <edges>
            std::istringstream iss(line);
            std::string p, sp;
            iss >> p >> sp >> outNodes >> outEdges;
        } else if (line[0] == 'a') {
            // a <from> <to> <cost>
            std::istringstream iss(line);
            char a;
            long u, v, w;
            iss >> a >> u >> v >> w;
            edges.emplace_back(u, v, w);
        }
    }
    return edges;
}

Roadmap Roadmap::LoadDIMACS(const std::string& costFile,
                            const std::vector<std::string>& resFiles) {
    size_t cdim = 1 + resFiles.size(); // primary cost + resource costs

    long nNodes = 0, nEdges = 0;
    auto costEdges = ParseDIMACSFile(costFile, nNodes, nEdges);

    // Parse resource files
    std::vector<std::vector<std::tuple<long, long, long>>> resEdges;
    for (auto& rf : resFiles) {
        long rn = 0, re = 0;
        resEdges.push_back(ParseDIMACSFile(rf, rn, re));
    }

    Roadmap rm;
    rm.Init(nNodes, cdim);

    // Add nodes (DIMACS is 1-based)
    for (long v = 1; v <= nNodes; v++) {
        rm.AddNode(v);
    }

    // Build edges with combined cost vectors
    for (size_t i = 0; i < costEdges.size(); i++) {
        auto [u, v, w] = costEdges[i];
        CostVector cv(0, cdim);
        cv[0] = w; // primary cost
        for (size_t r = 0; r < resEdges.size(); r++) {
            auto [ru, rv, rw] = resEdges[r][i];
            cv[r + 1] = rw;
        }
        rm.AddEdge(u, v, cv);
    }

    return rm;
}

} // namespace rcspp
