#include "frontier/AVLTree.h"
#include "graph/Graph.h"
#include <stdexcept>

namespace rcspp {

AVLNode* NewAVLNode(long id) {
    AVLNode* n = new AVLNode();
    n->id = id;
    n->left = nullptr;
    n->right = nullptr;
    n->h = 1;
    return n;
}

AVLNode* RightRotate(AVLNode* y) {
    AVLNode* x = y->left;
    AVLNode* t2 = x->right;
    x->right = y;
    y->left = t2;
    y->h = MaxH(H(y->left), H(y->right)) + 1;
    x->h = MaxH(H(x->left), H(x->right)) + 1;
    return x;
}

AVLNode* LeftRotate(AVLNode* x) {
    AVLNode* y = x->right;
    AVLNode* t2 = y->left;
    y->left = x;
    x->right = t2;
    x->h = MaxH(H(x->left), H(x->right)) + 1;
    y->h = MaxH(H(y->left), H(y->right)) + 1;
    return y;
}

int GetBalanceFactor(AVLNode* n) {
    if (!n) return 0;
    return static_cast<int>(H(n->left)) - static_cast<int>(H(n->right));
}

// --- AVLTree template implementation ---

template <typename KeyType>
AVLTree<KeyType>::AVLTree() {}

template <typename KeyType>
AVLTree<KeyType>::~AVLTree() {
    _deleteAll(root_);
}

template <typename KeyType>
void AVLTree<KeyType>::Add(KeyType k, long id) {
    root_ = _insert(root_, k, id);
}

template <typename KeyType>
AVLNode AVLTree<KeyType>::Find(KeyType k) {
    AVLNode* p = _find(root_, k);
    if (!p) return AVLNode();
    return *p;
}

template <typename KeyType>
int AVLTree<KeyType>::FindMaxLess(KeyType k, KeyType* out, bool if_equal, long* out_id) {
    long ref = -1;
    _findMaxLess(root_, k, &ref, if_equal);
    if (ref < 0) return 0;
    *out = keys_[ref];
    if (out_id) *out_id = ref;
    return 1;
}

template <typename KeyType>
int AVLTree<KeyType>::FindMinMore(KeyType k, KeyType* out, bool if_equal, long* out_id) {
    long ref = -1;
    _findMinMore(root_, k, &ref, if_equal);
    if (ref < 0) return 0;
    *out = keys_[ref];
    if (out_id) *out_id = ref;
    return 1;
}

template <typename KeyType>
void AVLTree<KeyType>::Delete(KeyType k) {
    root_ = _delete(root_, k);
}

template <typename KeyType>
void AVLTree<KeyType>::Clear() {
    keys_.clear();
    _deleteAll(root_);
    root_ = nullptr;
    idGen_ = 0;
    size_ = 0;
}

template <typename KeyType>
size_t AVLTree<KeyType>::Size() const {
    return size_;
}

template <typename KeyType>
void AVLTree<KeyType>::ToSortedVector(
    std::vector<KeyType>* out, std::vector<long>* out_id,
    std::unordered_set<long>* skip_set) {
    _inOrder(root_, out, out_id, skip_set);
}

template <typename KeyType>
AVLNode* AVLTree<KeyType>::_insert(AVLNode* n, KeyType k, long id0) {
    if (!n) {
        auto nn = NewAVLNode(id0);
        if (id0 < 0) nn->id = _genId();
        keys_[nn->id] = k;
        size_++;
        return nn;
    }

    if (k < keys_[n->id]) {
        n->left = _insert(n->left, k, id0);
    } else if (k > keys_[n->id]) {
        n->right = _insert(n->right, k, id0);
    } else {
        return n; // duplicate key
    }

    n->h = MaxH(H(n->left), H(n->right)) + 1;
    int b = GetBalanceFactor(n);

    // LL
    if (b > 1 && n->left && k < keys_[n->left->id])
        return RightRotate(n);
    // RR
    if (b < -1 && n->right && k > keys_[n->right->id])
        return LeftRotate(n);
    // LR
    if (b > 1 && n->left && k > keys_[n->left->id]) {
        n->left = LeftRotate(n->left);
        return RightRotate(n);
    }
    // RL
    if (b < -1 && n->right && k < keys_[n->right->id]) {
        n->right = RightRotate(n->right);
        return LeftRotate(n);
    }
    return n;
}

template <typename KeyType>
AVLNode* AVLTree<KeyType>::_find(AVLNode* n, KeyType k) {
    if (!n) return nullptr;
    if (k < keys_[n->id]) return _find(n->left, k);
    if (k > keys_[n->id]) return _find(n->right, k);
    return n;
}

template <typename KeyType>
void AVLTree<KeyType>::_findMaxLess(AVLNode* n, KeyType k, long* ref, bool if_equal) {
    if (!n) return;
    if (keys_[n->id] < k) {
        if (*ref == -1 || keys_[n->id] > keys_[*ref])
            *ref = n->id;
        _findMaxLess(n->right, k, ref, if_equal);
    } else if (keys_[n->id] > k) {
        _findMaxLess(n->left, k, ref, if_equal);
    } else {
        if (if_equal) {
            *ref = n->id;
        } else {
            _findMaxLess(n->left, k, ref, if_equal);
        }
    }
}

template <typename KeyType>
void AVLTree<KeyType>::_findMinMore(AVLNode* n, KeyType k, long* ref, bool if_equal) {
    if (!n) return;
    if (keys_[n->id] > k) {
        if (*ref == -1 || keys_[n->id] < keys_[*ref])
            *ref = n->id;
        _findMinMore(n->left, k, ref, if_equal);
    } else if (keys_[n->id] < k) {
        _findMinMore(n->right, k, ref, if_equal);
    } else {
        if (if_equal) {
            *ref = n->id;
        } else {
            _findMinMore(n->right, k, ref, if_equal);
        }
    }
}

template <typename KeyType>
AVLNode* AVLTree<KeyType>::_findMin(AVLNode* n) {
    if (!n) return nullptr;
    while (n->left) n = n->left;
    return n;
}

template <typename KeyType>
AVLNode* AVLTree<KeyType>::_delete(AVLNode* n, KeyType k) {
    if (!n) return nullptr;

    if (k < keys_[n->id]) {
        n->left = _delete(n->left, k);
    } else if (k > keys_[n->id]) {
        n->right = _delete(n->right, k);
    } else {
        if (!n->left || !n->right) {
            AVLNode* child = n->left ? n->left : n->right;
            if (!child) {
                delete n;
                size_--;
                return nullptr;
            } else {
                long oldId = n->id;
                *n = *child;
                if (child == n->left) n->left = nullptr;
                else n->right = nullptr;
                delete child;
                size_--;
            }
        } else {
            AVLNode* successor = _findMin(n->right);
            n->id = successor->id;
            n->right = _delete(n->right, keys_[successor->id]);
        }
    }

    if (!n) return nullptr;

    n->h = 1 + MaxH(H(n->left), H(n->right));
    int b = GetBalanceFactor(n);

    if (b > 1 && GetBalanceFactor(n->left) >= 0)
        return RightRotate(n);
    if (b > 1 && GetBalanceFactor(n->left) < 0) {
        n->left = LeftRotate(n->left);
        return RightRotate(n);
    }
    if (b < -1 && GetBalanceFactor(n->right) <= 0)
        return LeftRotate(n);
    if (b < -1 && GetBalanceFactor(n->right) > 0) {
        n->right = RightRotate(n->right);
        return LeftRotate(n);
    }
    return n;
}

template <typename KeyType>
void AVLTree<KeyType>::_deleteAll(AVLNode* n) {
    if (!n) return;
    _deleteAll(n->left);
    _deleteAll(n->right);
    delete n;
}

template <typename KeyType>
void AVLTree<KeyType>::_inOrder(AVLNode* n, std::vector<KeyType>* out,
                                std::vector<long>* out_id,
                                std::unordered_set<long>* skip_set) {
    if (!n) return;
    _inOrder(n->left, out, out_id, skip_set);
    if (!skip_set || skip_set->find(n->id) == skip_set->end()) {
        if (out) out->push_back(keys_[n->id]);
        if (out_id) out_id->push_back(n->id);
    }
    _inOrder(n->right, out, out_id, skip_set);
}

// Explicit instantiation
template class AVLTree<CostVector>;
template class AVLTree<long>;

} // namespace rcspp
