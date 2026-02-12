#include "frontier/ParetoFrontier.h"
#include <sstream>

namespace rcspp {

// --- Label ---

std::ostream& operator<<(std::ostream& os, const Label& l) {
    os << "{id:" << l.id << ",v:" << l.v
       << ",g:" << l.g.ToStr() << ",f:" << l.f.ToStr() << "}";
    return os;
}

// --- KOATree ---

KOATree::KOATree() {}
KOATree::~KOATree() {}

bool KOATree::Check(CostVector v) {
    return _check(root_, v);
}

bool KOATree::_check(AVLNode* n, const CostVector& k) {
    if (!n) return false;

    if (EpsDominance(keys_[n->id], k)) {
        return true; // n dominates k
    }

    if (keys_[n->id] > k) {
        return _check(n->left, k);
    } else {
        if (_check(n->left, k)) return true;
        return _check(n->right, k);
    }
}

void KOATree::Filter(CostVector v) {
    std::unordered_set<long> filtered;
    root_ = _filter(root_, v, &filtered);
    if (!filtered.empty()) {
        _rebuildTree(&filtered);
    }
}

AVLNode* KOATree::_filter(AVLNode* n, const CostVector& k,
                           std::unordered_set<long>* filtered) {
    if (!n) return nullptr;

    if (keys_[n->id] < k) {
        n->right = _filter(n->right, k, filtered);
    } else {
        n->left = _filter(n->left, k, filtered);
        n->right = _filter(n->right, k, filtered);
    }

    if (EpsDominance(k, keys_[n->id])) {
        if (filtered) filtered->insert(n->id);
    }
    return n;
}

void KOATree::_rebuildTree(std::unordered_set<long>* skip) {
    std::vector<long> ids;
    ToSortedVector(nullptr, &ids, skip);

    size_ = 0;
    AVLNode* newRoot = _rebuildMethod(ids, 0, static_cast<long>(ids.size()) - 1);
    _deleteAll(root_);
    root_ = newRoot;
}

AVLNode* KOATree::_rebuildMethod(std::vector<long>& ids, long start, long end) {
    if (start > end) return nullptr;
    long mid = (start + end) / 2;
    AVLNode* n = NewAVLNode(ids[mid]);
    size_++;
    n->left = _rebuildMethod(ids, start, mid - 1);
    n->right = _rebuildMethod(ids, mid + 1, end);
    n->h = 1 + MaxH(H(n->left), H(n->right));
    return n;
}

// --- ParetoFrontier ---

ParetoFrontier::ParetoFrontier() {}
ParetoFrontier::~ParetoFrontier() {}

CostVector ParetoFrontier::project(CostVector v) {
    CostVector out;
    for (size_t i = 1; i < v.size(); i++) {
        out.push_back(v[i]);
    }
    return out;
}

bool ParetoFrontier::Check(CostVector g) {
    CostVector pg = doTruncation_ ? project(g) : g;
    return tree_.Check(pg);
}

void ParetoFrontier::Update(Label l) {
    CostVector pg = doTruncation_ ? project(l.g) : l.g;

    if (tree_.Size() == 0) {
        tree_.Add(pg);
        labelIds.insert(l.id);
        return;
    }

    labelIds.insert(l.id);
    tree_.Filter(pg);
    tree_.Add(pg);
}

} // namespace rcspp
