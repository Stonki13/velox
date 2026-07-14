#pragma once
#include "velox/math.h"
#include <cstdint>
#include <vector>

namespace velox {

// Incremental dynamic AABB tree (Box2D b2DynamicTree style): leaves store fat
// AABBs so slowly moving bodies rarely reinsert, inner nodes are balanced with
// height-based rotations, and insertion descends by a surface-area cost
// heuristic. Deterministic: identical operation sequences produce identical
// trees, and pair/query output is post-sorted by callers where order matters.
class AabbTree {
public:
    static constexpr int32_t kNull = -1;

    // Extra margin applied to stored leaf bounds; a proxy only reinserts when
    // its tight bounds escape the fat bounds.
    static constexpr float kFatMargin = 0.1f;

    int32_t insert(const Vec3& lo, const Vec3& hi, uint32_t userData) {
        int32_t leaf = allocateNode();
        Vec3 margin{kFatMargin, kFatMargin, kFatMargin};
        nodes_[leaf].lo = lo - margin;
        nodes_[leaf].hi = hi + margin;
        nodes_[leaf].userData = userData;
        nodes_[leaf].height = 0;
        insertLeaf(leaf);
        return leaf;
    }

    void remove(int32_t proxy) {
        removeLeaf(proxy);
        freeNode(proxy);
    }

    // Updates a proxy's bounds; reinserts only if the tight bounds escaped the
    // stored fat bounds. Returns true when a reinsert happened.
    bool move(int32_t proxy, const Vec3& lo, const Vec3& hi) {
        Node& node = nodes_[proxy];
        if (lo.x >= node.lo.x && lo.y >= node.lo.y && lo.z >= node.lo.z &&
            hi.x <= node.hi.x && hi.y <= node.hi.y && hi.z <= node.hi.z)
            return false;
        removeLeaf(proxy);
        Vec3 margin{kFatMargin, kFatMargin, kFatMargin};
        node.lo = lo - margin;
        node.hi = hi + margin;
        insertLeaf(proxy);
        return true;
    }

    void updateUserData(int32_t proxy, uint32_t userData) {
        nodes_[proxy].userData = userData;
    }

    void clear() {
        nodes_.clear();
        root_ = kNull;
        freeList_ = kNull;
    }

    bool empty() const { return root_ == kNull; }

    // Visits every leaf whose fat AABB overlaps [lo, hi].
    template <typename F>
    void query(const Vec3& lo, const Vec3& hi, F&& visit) const {
        if (root_ == kNull) return;
        int32_t stack[64];
        int sp = 0;
        stack[sp++] = root_;
        while (sp > 0) {
            const Node& node = nodes_[stack[--sp]];
            if (hi.x < node.lo.x || lo.x > node.hi.x ||
                hi.y < node.lo.y || lo.y > node.hi.y ||
                hi.z < node.lo.z || lo.z > node.hi.z)
                continue;
            if (node.isLeaf()) {
                visit(node.userData);
            } else if (sp + 2 <= 64) {
                stack[sp++] = node.child1;
                stack[sp++] = node.child2;
            }
        }
    }

    // Visits every leaf whose fat AABB the segment origin + dir*t, t in
    // [0, maxDist], passes through. `visit` may return a smaller maxDist to
    // shrink the remaining search (closest-hit raycasts).
    template <typename F>
    void raySegment(const Vec3& origin, const Vec3& dir, float maxDist,
                    F&& visit) const {
        if (root_ == kNull) return;
        int32_t stack[64];
        int sp = 0;
        stack[sp++] = root_;
        while (sp > 0) {
            const Node& node = nodes_[stack[--sp]];
            if (!segmentHitsAabb(origin, dir, maxDist, node.lo, node.hi))
                continue;
            if (node.isLeaf()) {
                float shrunk = visit(node.userData);
                if (shrunk < maxDist) maxDist = shrunk;
            } else if (sp + 2 <= 64) {
                stack[sp++] = node.child1;
                stack[sp++] = node.child2;
            }
        }
    }

private:
    struct Node {
        Vec3 lo, hi;
        int32_t parent = kNull;
        int32_t child1 = kNull, child2 = kNull; // child1 doubles as free-list next
        int32_t height = 0;                     // -1 when on the free list
        uint32_t userData = 0;

        bool isLeaf() const { return child1 == kNull; }
    };

    static bool segmentHitsAabb(const Vec3& o, const Vec3& d, float maxDist,
                                const Vec3& lo, const Vec3& hi) {
        float t0 = 0.0f, t1 = maxDist;
        const float ov[3] = {o.x, o.y, o.z}, dv[3] = {d.x, d.y, d.z};
        const float lov[3] = {lo.x, lo.y, lo.z}, hiv[3] = {hi.x, hi.y, hi.z};
        for (int i = 0; i < 3; ++i) {
            if (dv[i] > -1e-9f && dv[i] < 1e-9f) {
                if (ov[i] < lov[i] || ov[i] > hiv[i]) return false;
                continue;
            }
            float inv = 1.0f / dv[i];
            float a = (lov[i] - ov[i]) * inv, b = (hiv[i] - ov[i]) * inv;
            if (a > b) { float t = a; a = b; b = t; }
            if (a > t0) t0 = a;
            if (b < t1) t1 = b;
            if (t0 > t1) return false;
        }
        return true;
    }

    static float area(const Vec3& lo, const Vec3& hi) {
        Vec3 d = hi - lo;
        return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
    }

    int32_t allocateNode() {
        if (freeList_ != kNull) {
            int32_t id = freeList_;
            freeList_ = nodes_[id].child1;
            nodes_[id] = Node{};
            return id;
        }
        nodes_.emplace_back();
        return static_cast<int32_t>(nodes_.size() - 1);
    }

    void freeNode(int32_t id) {
        nodes_[id].child1 = freeList_;
        nodes_[id].height = -1;
        freeList_ = id;
    }

    void insertLeaf(int32_t leaf) {
        if (root_ == kNull) {
            root_ = leaf;
            nodes_[leaf].parent = kNull;
            return;
        }

        // Descend towards the sibling with the lowest combined-surface cost.
        Vec3 lo = nodes_[leaf].lo, hi = nodes_[leaf].hi;
        int32_t index = root_;
        while (!nodes_[index].isLeaf()) {
            const Node& node = nodes_[index];
            float combined = area(vmin(lo, node.lo), vmax(hi, node.hi));
            float cost = 2.0f * combined;
            float inherit = 2.0f * (combined - area(node.lo, node.hi));

            auto childCost = [&](int32_t child) {
                const Node& c = nodes_[child];
                float merged = area(vmin(lo, c.lo), vmax(hi, c.hi));
                return c.isLeaf() ? merged + inherit
                                  : merged - area(c.lo, c.hi) + inherit;
            };
            float cost1 = childCost(node.child1);
            float cost2 = childCost(node.child2);
            if (cost < cost1 && cost < cost2) break;
            index = cost1 < cost2 ? node.child1 : node.child2;
        }

        // Splice a new parent above the chosen sibling.
        int32_t sibling = index;
        int32_t oldParent = nodes_[sibling].parent;
        int32_t newParent = allocateNode();
        nodes_[newParent].parent = oldParent;
        nodes_[newParent].lo = vmin(lo, nodes_[sibling].lo);
        nodes_[newParent].hi = vmax(hi, nodes_[sibling].hi);
        nodes_[newParent].height = nodes_[sibling].height + 1;
        nodes_[newParent].child1 = sibling;
        nodes_[newParent].child2 = leaf;
        nodes_[sibling].parent = newParent;
        nodes_[leaf].parent = newParent;
        if (oldParent == kNull) {
            root_ = newParent;
        } else if (nodes_[oldParent].child1 == sibling) {
            nodes_[oldParent].child1 = newParent;
        } else {
            nodes_[oldParent].child2 = newParent;
        }

        refitUpwards(nodes_[leaf].parent);
    }

    void removeLeaf(int32_t leaf) {
        if (leaf == root_) {
            root_ = kNull;
            return;
        }
        int32_t parent = nodes_[leaf].parent;
        int32_t grand = nodes_[parent].parent;
        int32_t sibling = nodes_[parent].child1 == leaf ? nodes_[parent].child2
                                                        : nodes_[parent].child1;
        if (grand == kNull) {
            root_ = sibling;
            nodes_[sibling].parent = kNull;
        } else {
            if (nodes_[grand].child1 == parent) nodes_[grand].child1 = sibling;
            else nodes_[grand].child2 = sibling;
            nodes_[sibling].parent = grand;
            refitUpwards(grand);
        }
        freeNode(parent);
    }

    // Walks to the root re-balancing and re-fitting bounds/heights.
    void refitUpwards(int32_t index) {
        while (index != kNull) {
            index = balance(index);
            Node& node = nodes_[index];
            const Node& c1 = nodes_[node.child1];
            const Node& c2 = nodes_[node.child2];
            node.lo = vmin(c1.lo, c2.lo);
            node.hi = vmax(c1.hi, c2.hi);
            node.height = 1 + (c1.height > c2.height ? c1.height : c2.height);
            index = node.parent;
        }
    }

    // AVL-style rotation when the subtree becomes lopsided. Returns the index
    // of the (possibly new) subtree root.
    int32_t balance(int32_t iA) {
        Node& A = nodes_[iA];
        if (A.isLeaf() || A.height < 2) return iA;

        int32_t iB = A.child1, iC = A.child2;
        int bal = nodes_[iC].height - nodes_[iB].height;
        if (bal > 1) return rotate(iA, iC, iB);   // C up
        if (bal < -1) return rotate(iA, iB, iC);  // B up
        return iA;
    }

    // Promotes child iUp of iA; the taller grandchild of iUp keeps its place.
    int32_t rotate(int32_t iA, int32_t iUp, int32_t iOther) {
        Node& A = nodes_[iA];
        Node& U = nodes_[iUp];
        int32_t iG1 = U.child1, iG2 = U.child2;

        U.child1 = iA;
        U.parent = A.parent;
        A.parent = iUp;
        if (U.parent != kNull) {
            if (nodes_[U.parent].child1 == iA) nodes_[U.parent].child1 = iUp;
            else nodes_[U.parent].child2 = iUp;
        } else {
            root_ = iUp;
        }

        int32_t iKeep = nodes_[iG1].height > nodes_[iG2].height ? iG1 : iG2;
        int32_t iDown = iKeep == iG1 ? iG2 : iG1;
        U.child2 = iKeep;
        // A keeps `other` and adopts the shorter grandchild.
        if (A.child1 == iUp) A.child1 = iDown;
        else A.child2 = iDown;
        (void)iOther;
        nodes_[iDown].parent = iA;

        const Node& a1 = nodes_[A.child1];
        const Node& a2 = nodes_[A.child2];
        A.lo = vmin(a1.lo, a2.lo);
        A.hi = vmax(a1.hi, a2.hi);
        A.height = 1 + (a1.height > a2.height ? a1.height : a2.height);
        const Node& u1 = nodes_[U.child1];
        const Node& u2 = nodes_[U.child2];
        U.lo = vmin(u1.lo, u2.lo);
        U.hi = vmax(u1.hi, u2.hi);
        U.height = 1 + (u1.height > u2.height ? u1.height : u2.height);
        return iUp;
    }

    std::vector<Node> nodes_;
    int32_t root_ = kNull;
    int32_t freeList_ = kNull;
};

} // namespace velox
