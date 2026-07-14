#pragma once
#include "aabb_tree.h"
#include "velox/body.h"

namespace velox {

// World's incremental broad-phase state (pimpl'd out of the public header).
// The tree holds one fat proxy per non-plane body; planes are unbounded and
// kept in their own list. `structureDirty` forces a full rebuild (body
// removal reshuffles dense indices; snapshots and origin shifts replace
// geometry wholesale); `touched` marks that body state may have changed via
// the public API since the last refit, so lazy users of the tree (queries)
// re-fit before trusting it.
struct BroadPhaseData {
    AabbTree tree;
    std::vector<int32_t> proxies;   // parallel to dense bodies; -1 for planes
    std::vector<BodyIndex> planes;
    bool structureDirty = true;
    bool touched = true;
    float lastDt = 1.0f / 60.0f;
};

} // namespace velox
