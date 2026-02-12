#ifndef THESIS_RCSPP_PARETO_FRONTIER_H
#define THESIS_RCSPP_PARETO_FRONTIER_H

#include "frontier/AVLTree.h"
#include "graph/Graph.h"
#include <unordered_set>

namespace rcspp {

struct Label {
    Label() {}
    Label(long id0, long v0, CostVector g0, CostVector f0)
        : id(id0), v(v0), g(g0), f(f0) {}
    long id = -1;
    long v = -1;
    CostVector g;
    CostVector f;
};

std::ostream& operator<<(std::ostream& os, const Label& l);

// KOA Tree: AVL tree with dominance-aware Check operation
class KOATree : public AVLTree<CostVector> {
public:
    KOATree();
    virtual ~KOATree();

    // Check if v is dominated by any key in the tree
    bool Check(CostVector v);

    // Remove keys dominated by v, then rebuild tree
    void Filter(CostVector v);

protected:
    bool _check(AVLNode* n, const CostVector& k);
    AVLNode* _filter(AVLNode* n, const CostVector& k, std::unordered_set<long>* filtered);
    void _rebuildTree(std::unordered_set<long>* skip);
    AVLNode* _rebuildMethod(std::vector<long>& ids, long start, long end);
};

// Pareto frontier at each vertex for dominance pruning
// Works with arbitrary dimension cost vectors (projects out first component)
class ParetoFrontier {
public:
    ParetoFrontier();
    ~ParetoFrontier();

    // Check if g is dominated by existing labels
    bool Check(CostVector g);

    // Update frontier with new label
    void Update(Label l);

    std::unordered_set<long> labelIds;

private:
    // Project: remove the first component of v
    CostVector project(CostVector v);

    KOATree tree_;
    bool doTruncation_ = true;
};

} // namespace rcspp

#endif // THESIS_RCSPP_PARETO_FRONTIER_H
