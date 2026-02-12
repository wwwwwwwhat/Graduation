#ifndef THESIS_RCSPP_ROADMAP_H
#define THESIS_RCSPP_ROADMAP_H

#include "graph/Graph.h"
#include <string>

namespace rcspp {

using AdjList = std::unordered_map<long, std::unordered_map<long, CostVector>>;

class Roadmap : public Graph {
public:
    Roadmap();
    virtual ~Roadmap() = default;

    void Init(long numNodes, size_t costDim);

    bool HasNode(long v) override;
    std::unordered_set<long> GetSuccs(long v) override;
    std::unordered_set<long> GetPreds(long v) override;
    CostVector GetCost(long u, long v) override;
    size_t GetCostDim() override;
    std::unordered_set<long> GetNodes() override;
    long GetNumberOfNodes() override;
    long GetNumberOfEdges() override;

    void AddNode(long v);
    void AddEdge(long u, long v, const CostVector& cost);
    bool HasEdge(long u, long v);

    // Load from DIMACS format files (1-based indexing in files)
    // costFile: primary cost, resFiles: additional resource cost files
    static Roadmap LoadDIMACS(const std::string& costFile,
                              const std::vector<std::string>& resFiles);

    AdjList adjlist;      // forward adjacency
    AdjList adjlistRev;   // reverse adjacency
    std::unordered_set<long> nodes;

private:
    long numNodes_ = 0;
    long numEdges_ = 0;
    size_t costDim_ = 0;
};

} // namespace rcspp

#endif // THESIS_RCSPP_ROADMAP_H
