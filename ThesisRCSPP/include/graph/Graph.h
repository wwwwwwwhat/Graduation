#ifndef THESIS_RCSPP_GRAPH_H
#define THESIS_RCSPP_GRAPH_H

#include <vector>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include <limits>

namespace rcspp {

// Multi-dimensional cost vector, wrapping std::vector<long>
struct CostVector : std::vector<long> {
    CostVector();
    CostVector(long val, size_t dim);
    CostVector(const std::vector<long>& v);

    CostVector operator+(const CostVector& rhs) const;
    CostVector& operator+=(const CostVector& rhs);
    CostVector operator-(const CostVector& rhs) const;
    bool operator==(const CostVector& rhs) const;

    // Lexicographic comparison: <0 means this<v, 0 means equal, >0 means this>v
    int CompareLexico(const CostVector& v) const;

    // Element-wise minimum
    CostVector ElemWiseMin(const CostVector& rhs) const;

    std::string ToStr() const;
};

std::ostream& operator<<(std::ostream& os, const CostVector& c);

// Check if v1 epsilon-dominates v2 (every component of v1 <= (1+eps)*v2)
bool EpsDominance(const CostVector& v1, const CostVector& v2, double eps = 0.0);

// Abstract graph interface
class Graph {
public:
    Graph() = default;
    virtual ~Graph() = default;

    virtual bool HasNode(long v) = 0;
    virtual std::unordered_set<long> GetSuccs(long v) = 0;
    virtual std::unordered_set<long> GetPreds(long v) = 0;
    virtual CostVector GetCost(long u, long v) = 0;
    virtual size_t GetCostDim() = 0;
    virtual std::unordered_set<long> GetNodes() = 0;
    virtual long GetNumberOfNodes() = 0;
    virtual long GetNumberOfEdges() = 0;
};

} // namespace rcspp

#endif // THESIS_RCSPP_GRAPH_H
