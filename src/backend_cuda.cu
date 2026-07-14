// CUDA backend: integration and the full narrow phase (GJK, manifolds, mesh
// BVH traversal) run on the GPU via the same VELOX_HD code the CPU uses.
// Contact and supported joint constraints use graph-colored device passes;
// small or unsupported joint workloads retain the lower-latency CPU path.
#include "narrowphase.h"
#include "joint_solver.h"
#include <cuda_runtime.h>
#include <thrust/device_ptr.h>
#include <thrust/sort.h>
#include <algorithm>
#include <cstdlib>
#include <cstdio>

namespace velox {

namespace {

#define VELOX_CUDA_CHECK(expr)                                                \
    do {                                                                      \
        cudaError_t err_ = (expr);                                            \
        if (err_ != cudaSuccess)                                              \
            std::fprintf(stderr, "velox cuda: %s at %s:%d\n",                 \
                         cudaGetErrorString(err_), __FILE__, __LINE__);       \
    } while (0)

__global__ void integrateKernel(Body* bodies, int n, Vec3 gravity, float dt) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    Body& b = bodies[i];
    if (!b.isDynamic() || b.asleep) return;
    b.velocity += (gravity * b.gravityScale + b.force * b.solverInvMass()) * dt;
    b.angularVelocity += b.invInertiaMul(b.torque) * dt;
    b.velocity *= 1.0f / (1.0f + b.linearDamping * dt);
    b.angularVelocity *= 1.0f / (1.0f + b.angularDamping * dt);
}

__global__ void integrateTransformsKernel(Body* bodies, int n, float dt) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    Body& b = bodies[i];
    if (b.isStatic() || b.asleep) return;
    b.advanceTransform(dt);
}

// Compact AABBs so the pair kernel can reject without touching Body structs.
// Planes and meshes get infinite bounds (they are collided by every pair).
struct Aabb { Vec3 lo, hi; };

__global__ void aabbKernel(const Body* bodies, int n, float dt, Aabb* aabbs,
                           float* sortKeys, BodyIndex* sortedIndices) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    const Body& b = bodies[i];
    if (b.shape == ShapeType::Plane || b.shape == ShapeType::Mesh) {
        aabbs[i] = {{-1e30f, -1e30f, -1e30f}, {1e30f, 1e30f, 1e30f}};
    } else {
        Aabb box;
        bodyAabb(b, dt, box.lo, box.hi);
        aabbs[i] = box;
    }
    sortKeys[i] = aabbs[i].lo.x;
    sortedIndices[i] = (BodyIndex)i;
}

// Stage 1: compact AABB-overlapping pairs from sorted X intervals. The atomic
// count is allowed to exceed pairCap; the host grows the buffer and reruns so
// broad-phase overflow never drops collision candidates.
__global__ void sweepCandidatesKernel(const Body* bodies, const Aabb* aabbs,
                                      const float* sortedKeys,
                                      const BodyIndex* sortedIndices, int n,
                                      uint64_t* pairs, int* pairCount,
                                      int pairCap) {
    int s = blockIdx.x * blockDim.x + threadIdx.x;
    if (s >= n) return;
    BodyIndex rawI = sortedIndices[s];
    for (int t = s + 1; t < n && sortedKeys[t] <= aabbs[rawI].hi.x; ++t) {
        BodyIndex rawJ = sortedIndices[t];
        BodyIndex i = rawI < rawJ ? rawI : rawJ;
        BodyIndex j = rawI < rawJ ? rawJ : rawI;
        if (!aabbOverlap(aabbs[i].lo, aabbs[i].hi, aabbs[j].lo, aabbs[j].hi)) continue;
        const Body& a = bodies[i];
        const Body& b = bodies[j];
        if ((a.isStatic() || a.asleep) && (b.isStatic() || b.asleep)) continue;
        if (!a.canCollideWith(b)) continue;

        int slot = atomicAdd(pairCount, 1);
        if (slot < pairCap) pairs[slot] = (uint64_t(i) << 32) | j;
    }
}

// Stage 2: expensive narrow phase is fully parallel across compact candidates.
__global__ void candidateContactsKernel(const Body* bodies,
                                        const uint64_t* pairs, int pairCount,
                                        MeshSoupView soup, float dt,
                                        Contact* out, int* outCount, int outCap) {
    int k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k >= pairCount) return;
    uint64_t pair = pairs[k];
    BodyIndex i = BodyIndex(pair >> 32), j = BodyIndex(pair);
    Contact buf[kMaxContactsPerPair];
    int count = collidePair(bodies[i], bodies[j], i, j, soup, dt,
                            buf, kMaxContactsPerPair);
    if (count == 0) return;
    int slot = atomicAdd(outCount, count);
    for (int c = 0; c < count && slot + c < outCap; ++c)
        out[slot + c] = buf[c];
}

// Reference all-pairs path retained for dense-scene fallback and controlled
// benchmarking. Compact AABBs keep rejected pairs cheap.
__global__ void allPairsContactsKernel(const Body* bodies, const Aabb* aabbs,
                                       int n, MeshSoupView soup, float dt,
                                       Contact* out, int* outCount, int outCap) {
    long long idx = blockIdx.x * (long long)blockDim.x + threadIdx.x;
    long long total = (long long)n * (n - 1) / 2;
    if (idx >= total) return;
    int i = (int)((2LL * n - 1 - sqrtf((float)((2LL * n - 1) *
        (2LL * n - 1) - 8LL * idx))) / 2);
    long long rowStart = (long long)i * n - (long long)i * (i + 1) / 2;
    int j = (int)(idx - rowStart + i + 1);
    while (j >= n) {
        ++i;
        rowStart = (long long)i * n - (long long)i * (i + 1) / 2;
        j = (int)(idx - rowStart + i + 1);
    }
    if (!aabbOverlap(aabbs[i].lo, aabbs[i].hi, aabbs[j].lo, aabbs[j].hi)) return;
    const Body& a = bodies[i];
    const Body& b = bodies[j];
    if ((a.isStatic() || a.asleep) && (b.isStatic() || b.asleep)) return;
    if (!a.canCollideWith(b)) return;
    Contact buf[kMaxContactsPerPair];
    int count = collidePair(a, b, (BodyIndex)i, (BodyIndex)j, soup, dt,
                            buf, kMaxContactsPerPair);
    if (count == 0) return;
    int slot = atomicAdd(outCount, count);
    for (int c = 0; c < count && slot + c < outCap; ++c) out[slot + c] = buf[c];
}

// Solves every contact of one color in parallel: within a color no two
// contacts share a dynamic body, so the writes cannot conflict.
__global__ void solveColorKernel(Body* bodies, Contact* contacts,
                                 int first, int count, float dt) {
    int k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k >= count) return;
    Contact& c = contacts[first + k];
    if (bodies[c.a].isSensor() || bodies[c.b].isSensor()) return;
    solveContact(bodies[c.a], bodies[c.b], c, dt);
}

__global__ void warmStartColorKernel(Body* bodies, Contact* contacts,
                                     int first, int count) {
    int k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k >= count) return;
    Contact& c = contacts[first + k];
    if (bodies[c.a].isSensor() || bodies[c.b].isSensor()) return;
    warmStartContact(bodies[c.a], bodies[c.b], c);
}

__global__ void resetJointsKernel(Joint* joints, int count,
                                  bool preserveBroken) {
    int k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k >= count || (preserveBroken && joints[k].broken)) return;
    joint_solver::reset(joints[k]);
}

__global__ void solveJointColorKernel(Body* bodies, Joint* joints,
                                      int first, int count, float dt) {
    int k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k >= count) return;
    Joint& joint = joints[first + k];
    if (joint.broken) return;
    Body& a = bodies[joint.a];
    Body& b = bodies[joint.b];
    if (a.asleep && b.asleep) return;
    joint_solver::solve(joint, a, b, dt);
}

__global__ void markBrokenJointsKernel(Joint* joints, int count, float dt) {
    int k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k >= count || joints[k].broken) return;
    Joint& joint = joints[k];
    float force = length(joint.reactionLinearImpulse) / dt;
    float torque = length(joint.reactionAngularImpulse) / dt;
    joint.broken = force > joint.breakForce || torque > joint.breakTorque;
}

class CudaBackend final : public Backend {
public:
    static CudaBackend* create() {
        int count = 0;
        if (cudaGetDeviceCount(&count) != cudaSuccess || count == 0) return nullptr;
        return new CudaBackend();
    }

    ~CudaBackend() override {
        // Graph instantiation may leave runtime work outside the solve stream;
        // drain the device before graph and allocation teardown.
        VELOX_CUDA_CHECK(cudaDeviceSynchronize());
        if (solveGraph_) cudaGraphExecDestroy(solveGraph_);
        if (jointGraph_) cudaGraphExecDestroy(jointGraph_);
        if (stream_) cudaStreamDestroy(stream_);
        release(dBodies_);
        release(dAabbs_);
        release(dSortKeys_);
        release(dSortedIndices_);
        release(dPairs_);
        release(dSolve_);
        release(dContacts_);
        release(dJoints_);
        release(dCount_);
        release(dVertices_);
        release(dIndices_);
        release(dMeshes_);
        release(dNodes_);
        release(dTriRefs_);
        release(dHullPts_);
        release(dCompoundChildren_);
    }

    const char* name() const override { return "cuda"; }

    void integrate(std::vector<Body>& bodies, const Vec3& gravity, float dt) override {
        int n = static_cast<int>(bodies.size());
        if (n == 0) return;
        uploadBodies(bodies);
        integrateKernel<<<(n + 255) / 256, 256>>>(dBodies_, n, gravity, dt);
        VELOX_CUDA_CHECK(cudaMemcpy(bodies.data(), dBodies_, n * sizeof(Body),
                                    cudaMemcpyDeviceToHost));
        bodiesDirty_ = false; // device copy is current
    }

    void findContacts(const std::vector<Body>& bodies, const MeshSoup& meshes,
                      float dt, std::vector<Contact>& out) override {
        out.clear();
        int n = static_cast<int>(bodies.size());
        if (n < 2) return;
        if (bodiesDirty_) uploadBodies(const_cast<std::vector<Body>&>(bodies));
        uploadMeshes(meshes);

        int cap = vmaxContacts(n);
        if ((size_t)cap * sizeof(Contact) > contactCap_) {
            release(dContacts_);
            VELOX_CUDA_CHECK(cudaMalloc(&dContacts_, cap * sizeof(Contact)));
            contactCap_ = cap * sizeof(Contact);
        }
        if (!dCount_) VELOX_CUDA_CHECK(cudaMalloc(&dCount_, sizeof(int)));
        VELOX_CUDA_CHECK(cudaMemset(dCount_, 0, sizeof(int)));

        if ((size_t)n * sizeof(Aabb) > aabbCap_) {
            release(dAabbs_);
            release(dSortKeys_);
            release(dSortedIndices_);
            VELOX_CUDA_CHECK(cudaMalloc(&dAabbs_, n * sizeof(Aabb)));
            VELOX_CUDA_CHECK(cudaMalloc(&dSortKeys_, n * sizeof(float)));
            VELOX_CUDA_CHECK(cudaMalloc(&dSortedIndices_, n * sizeof(BodyIndex)));
            aabbCap_ = n * sizeof(Aabb);
        }
        aabbKernel<<<(n + 255) / 256, 256>>>(dBodies_, n, dt, dAabbs_,
                                             dSortKeys_, dSortedIndices_);
        MeshSoupView soup{dVertices_, dIndices_, dMeshes_, dNodes_, dTriRefs_,
                          dHullPts_, dCompoundChildren_};
        int threads = 256;
        // The brute-force kernel has excellent occupancy for small scenes;
        // radix sorting and candidate compaction pay off only once quadratic
        // rejection dominates. Keep both and switch at the measured crossover.
        if (n < 4096 || std::getenv("VELOX_CUDA_ALL_PAIRS")) {
            long long pairs = (long long)n * (n - 1) / 2;
            long long blocks = (pairs + threads - 1) / threads;
            allPairsContactsKernel<<<(unsigned)blocks, threads>>>(
                dBodies_, dAabbs_, n, soup, dt, dContacts_, dCount_, cap);
            int count = 0;
            VELOX_CUDA_CHECK(cudaMemcpy(&count, dCount_, sizeof(int), cudaMemcpyDeviceToHost));
            count = count < cap ? count : cap;
            out.resize(count);
            if (count > 0)
                VELOX_CUDA_CHECK(cudaMemcpy(out.data(), dContacts_, count * sizeof(Contact),
                                            cudaMemcpyDeviceToHost));
            bodiesDirty_ = true;
            return;
        }

        thrust::device_ptr<float> keys(dSortKeys_);
        thrust::device_ptr<BodyIndex> indices(dSortedIndices_);
        thrust::sort_by_key(keys, keys + n, indices);
        int candidateCap = static_cast<int>(pairCap_ / sizeof(uint64_t));
        if (candidateCap == 0) {
            candidateCap = n * 128 + 1024;
            VELOX_CUDA_CHECK(cudaMalloc(&dPairs_, candidateCap * sizeof(uint64_t)));
            pairCap_ = candidateCap * sizeof(uint64_t);
        }
        VELOX_CUDA_CHECK(cudaMemset(dCount_, 0, sizeof(int)));
        sweepCandidatesKernel<<<(n + threads - 1) / threads, threads>>>(
            dBodies_, dAabbs_, dSortKeys_, dSortedIndices_, n,
            dPairs_, dCount_, candidateCap);
        int candidateCount = 0;
        VELOX_CUDA_CHECK(cudaMemcpy(&candidateCount, dCount_, sizeof(int),
                                    cudaMemcpyDeviceToHost));
        if (candidateCount > candidateCap) {
            release(dPairs_);
            candidateCap = candidateCount;
            VELOX_CUDA_CHECK(cudaMalloc(&dPairs_, candidateCap * sizeof(uint64_t)));
            pairCap_ = candidateCap * sizeof(uint64_t);
            VELOX_CUDA_CHECK(cudaMemset(dCount_, 0, sizeof(int)));
            sweepCandidatesKernel<<<(n + threads - 1) / threads, threads>>>(
                dBodies_, dAabbs_, dSortKeys_, dSortedIndices_, n,
                dPairs_, dCount_, candidateCap);
        }
        VELOX_CUDA_CHECK(cudaMemset(dCount_, 0, sizeof(int)));
        if (candidateCount > 0)
            candidateContactsKernel<<<(candidateCount + threads - 1) / threads, threads>>>(
                dBodies_, dPairs_, candidateCount, soup, dt, dContacts_, dCount_, cap);
        int count = 0;
        VELOX_CUDA_CHECK(cudaMemcpy(&count, dCount_, sizeof(int), cudaMemcpyDeviceToHost));
        count = count < cap ? count : cap;
        out.resize(count);
        if (count > 0)
            VELOX_CUDA_CHECK(cudaMemcpy(out.data(), dContacts_, count * sizeof(Contact),
                                        cudaMemcpyDeviceToHost));
        bodiesDirty_ = true; // host may mutate bodies before the solve call
    }

    void solveVelocities(std::vector<Body>& bodies,
                         std::vector<Contact>& contacts, float dt,
                         bool warmStart) override {
        int n = static_cast<int>(bodies.size());
        int m = static_cast<int>(contacts.size());
        if (m == 0) return;
        if (bodiesDirty_) uploadBodies(bodies);

        // warmStart marks the first substep of a step: sort, color, and
        // upload contacts once; later substeps reuse the device-resident
        // contacts (whose impulses keep accumulating) and cached coloring.
        if (warmStart) {
            // The contact kernel appends via atomics, so arrival order is
            // random each frame. Sort deterministically first: otherwise the
            // coloring (and thus the solve order) changes every frame, and
            // that noise is enough to walk a tall stack off balance.
            std::sort(contacts.begin(), contacts.end(),
                      [](const Contact& x, const Contact& y) {
                          uint64_t kx = (uint64_t)x.a << 32 | x.b;
                          uint64_t ky = (uint64_t)y.a << 32 | y.b;
                          if (kx != ky) return kx < ky;
                          if (x.point.x != y.point.x) return x.point.x < y.point.x;
                          if (x.point.y != y.point.y) return x.point.y < y.point.y;
                          return x.point.z < y.point.z;
                      });

            // Greedy first-free-color coloring on the host: each contact takes
            // the smallest color unused by either dynamic endpoint (bitmask up
            // to 64 colors, overflow spills sequentially above). This yields
            // roughly max-contacts-per-body colors — an order of magnitude
            // fewer kernel launches than chain-growth schemes. Static bodies
            // are never written and don't constrain colors.
            colorMask_.assign(n, 0);
            nextColor_.assign(n, 64);
            colorOf_.resize(m);
            numColors_ = 0;
            for (int k = 0; k < m; ++k) {
                const Contact& c = contacts[k];
                bool sensor = bodies[c.a].isSensor() || bodies[c.b].isSensor();
                bool sa = sensor || !bodies[c.a].isDynamic();
                bool sb = sensor || !bodies[c.b].isDynamic();
                uint64_t used = (sa ? 0 : colorMask_[c.a]) | (sb ? 0 : colorMask_[c.b]);
                int color;
                if (~used) {
                    color = 0;
                    while (used & (1ull << color)) ++color;
                } else {
                    int na = sa ? 64 : nextColor_[c.a];
                    int nb = sb ? 64 : nextColor_[c.b];
                    color = na > nb ? na : nb;
                    if (!sa) nextColor_[c.a] = color + 1;
                    if (!sb) nextColor_[c.b] = color + 1;
                }
                if (color < 64) {
                    if (!sa) colorMask_[c.a] |= 1ull << color;
                    if (!sb) colorMask_[c.b] |= 1ull << color;
                }
                colorOf_[k] = color;
                if (color + 1 > numColors_) numColors_ = color + 1;
            }

            // Counting sort contacts into color order (host copy too, so the
            // eventual fetchImpulses lines up 1:1 with the device layout).
            colorStart_.assign(numColors_ + 1, 0);
            for (int k = 0; k < m; ++k) ++colorStart_[colorOf_[k] + 1];
            for (int c = 0; c < numColors_; ++c) colorStart_[c + 1] += colorStart_[c];
            sorted_.resize(m);
            fill_ = colorStart_;
            for (int k = 0; k < m; ++k) sorted_[fill_[colorOf_[k]]++] = contacts[k];
            contacts = sorted_;

            if ((size_t)m * sizeof(Contact) > solveCap_) {
                release(dSolve_);
                VELOX_CUDA_CHECK(cudaMalloc(&dSolve_, m * sizeof(Contact)));
                solveCap_ = m * sizeof(Contact);
            }
            VELOX_CUDA_CHECK(cudaMemcpy(dSolve_, contacts.data(), m * sizeof(Contact),
                                        cudaMemcpyHostToDevice));
            solveCount_ = m;
        }

        if (!stream_) VELOX_CUDA_CHECK(cudaStreamCreate(&stream_));

        if (warmStart) {
            for (int c = 0; c < numColors_; ++c) {
                int first = colorStart_[c], count = colorStart_[c + 1] - first;
                warmStartColorKernel<<<(count + 127) / 128, 128, 0, stream_>>>(
                    dBodies_, dSolve_, first, count);
            }
            // Thousands of tiny per-color launches per frame are dominated by
            // launch overhead. Capture the whole iteration sweep as a CUDA
            // graph once per step and replay it each substep.
            if (solveGraph_) { cudaGraphExecDestroy(solveGraph_); solveGraph_ = nullptr; }
            // Colored order converges slower per sweep than sequential order
            // (color 0 solves one contact of every pair Jacobi-style), so run
            // twice the sweeps — with the graph replay they are nearly free.
            constexpr int kGpuIterations = 2 * kVelocityIterations;
            cudaGraph_t graph;
            VELOX_CUDA_CHECK(cudaStreamBeginCapture(stream_, cudaStreamCaptureModeGlobal));
            for (int iter = 0; iter < kGpuIterations; ++iter)
                for (int c = 0; c < numColors_; ++c) {
                    int first = colorStart_[c], count = colorStart_[c + 1] - first;
                    solveColorKernel<<<(count + 127) / 128, 128, 0, stream_>>>(
                        dBodies_, dSolve_, first, count, dt);
                }
            VELOX_CUDA_CHECK(cudaStreamEndCapture(stream_, &graph));
            VELOX_CUDA_CHECK(cudaGraphInstantiate(&solveGraph_, graph, nullptr, nullptr, 0));
            VELOX_CUDA_CHECK(cudaGraphDestroy(graph));
        }
        VELOX_CUDA_CHECK(cudaGraphLaunch(solveGraph_, stream_));

        VELOX_CUDA_CHECK(cudaMemcpyAsync(bodies.data(), dBodies_, n * sizeof(Body),
                                         cudaMemcpyDeviceToHost, stream_));
        VELOX_CUDA_CHECK(cudaStreamSynchronize(stream_));
        bodiesDirty_ = true; // host integrates positions before the next call
    }

    bool advanceSubsteps(std::vector<Body>& bodies,
                         std::vector<Contact>& contacts,
                         std::vector<Joint>& joints,
                         const Vec3& gravity, float dt,
                         int substeps) override {
        // Graph construction dominates small constraint sets; the CPU path is
        // lower latency until enough independent rows amortize the capture.
        constexpr size_t kDeviceJointThreshold = 64;
        if (!joints.empty() && joints.size() < kDeviceJointThreshold)
            return false;
        for (const Joint& joint : joints)
            if (!joint_solver::supported(joint)) return false;

        for (const Joint& joint : joints) {
            if (bodies[joint.a].asleep == bodies[joint.b].asleep) continue;
            bodies[joint.a].asleep = bodies[joint.b].asleep = 0;
            bodies[joint.a].sleepTimer = bodies[joint.b].sleepTimer = 0.0f;
        }

        bool hasContacts = !contacts.empty();
        solveVelocities(bodies, contacts, dt, true);
        if (bodies.empty()) return true;

        int n = static_cast<int>(bodies.size());
        uploadBodies(bodies);
        prepareJoints(bodies, joints, dt);
        if (!stream_) VELOX_CUDA_CHECK(cudaStreamCreate(&stream_));
        for (int s = 0; s < substeps; ++s) {
            if (s > 0)
            integrateKernel<<<(n + 255) / 256, 256, 0, stream_>>>(
                dBodies_, n, gravity, dt);
            if (s > 0 && hasContacts && solveGraph_)
                VELOX_CUDA_CHECK(cudaGraphLaunch(solveGraph_, stream_));
            if (!joints.empty()) {
                int jointCount = static_cast<int>(joints.size());
                resetJointsKernel<<<(jointCount + 127) / 128, 128, 0, stream_>>>(
                    dJoints_, jointCount, s > 0);
                VELOX_CUDA_CHECK(cudaGraphLaunch(jointGraph_, stream_));
            }
            integrateTransformsKernel<<<(n + 255) / 256, 256, 0, stream_>>>(
                dBodies_, n, dt);
        }
        VELOX_CUDA_CHECK(cudaMemcpyAsync(bodies.data(), dBodies_, n * sizeof(Body),
                                         cudaMemcpyDeviceToHost, stream_));
        VELOX_CUDA_CHECK(cudaStreamSynchronize(stream_));
        if (!joints.empty()) {
            VELOX_CUDA_CHECK(cudaMemcpy(jointSorted_.data(), dJoints_,
                                       joints.size() * sizeof(Joint),
                                       cudaMemcpyDeviceToHost));
            for (size_t i = 0; i < joints.size(); ++i)
                joints[jointOrder_[i]] = jointSorted_[i];
        }
        bodiesDirty_ = true;
        return true;
    }

    void fetchImpulses(std::vector<Contact>& contacts) override {
        if (solveCount_ == 0 || contacts.size() != (size_t)solveCount_) return;
        VELOX_CUDA_CHECK(cudaMemcpy(contacts.data(), dSolve_,
                                    solveCount_ * sizeof(Contact),
                                    cudaMemcpyDeviceToHost));
    }

    void invalidateCaches() override {
        bodiesDirty_ = true;
        meshVerts_ = meshNodes_ = hullPts_ = compoundChildren_ = (size_t)-1;
        solveCount_ = 0;
        if (solveGraph_) {
            cudaGraphExecDestroy(solveGraph_);
            solveGraph_ = nullptr;
        }
        if (jointGraph_) {
            cudaGraphExecDestroy(jointGraph_);
            jointGraph_ = nullptr;
        }
    }

private:
    template <typename T> void release(T*& p) {
        if (p) { cudaFree(p); p = nullptr; }
    }

    void uploadBodies(std::vector<Body>& bodies) {
        size_t bytes = bodies.size() * sizeof(Body);
        if (bytes > bodyCap_) {
            release(dBodies_);
            VELOX_CUDA_CHECK(cudaMalloc(&dBodies_, bytes));
            bodyCap_ = bytes;
        }
        VELOX_CUDA_CHECK(cudaMemcpy(dBodies_, bodies.data(), bytes, cudaMemcpyHostToDevice));
    }

    void prepareJoints(const std::vector<Body>& bodies,
                       const std::vector<Joint>& joints, float dt) {
        int count = static_cast<int>(joints.size());
        if (count == 0) return;

        std::vector<int> previousColorStart = jointColorStart_;
        std::vector<int> previousOrder = jointOrder_;

        jointColorMask_.assign(bodies.size(), 0);
        jointNextColor_.assign(bodies.size(), 64);
        jointColorOf_.resize(count);
        jointNumColors_ = 0;
        for (int k = 0; k < count; ++k) {
            const Joint& joint = joints[k];
            bool staticA = !bodies[joint.a].isDynamic();
            bool staticB = !bodies[joint.b].isDynamic();
            uint64_t used = (staticA ? 0 : jointColorMask_[joint.a]) |
                            (staticB ? 0 : jointColorMask_[joint.b]);
            int color;
            if (~used) {
                color = 0;
                while (used & (1ull << color)) ++color;
            } else {
                int nextA = staticA ? 64 : jointNextColor_[joint.a];
                int nextB = staticB ? 64 : jointNextColor_[joint.b];
                color = nextA > nextB ? nextA : nextB;
                if (!staticA) jointNextColor_[joint.a] = color + 1;
                if (!staticB) jointNextColor_[joint.b] = color + 1;
            }
            if (color < 64) {
                if (!staticA) jointColorMask_[joint.a] |= 1ull << color;
                if (!staticB) jointColorMask_[joint.b] |= 1ull << color;
            }
            jointColorOf_[k] = color;
            if (color + 1 > jointNumColors_) jointNumColors_ = color + 1;
        }

        jointColorStart_.assign(jointNumColors_ + 1, 0);
        for (int color : jointColorOf_) ++jointColorStart_[color + 1];
        for (int c = 0; c < jointNumColors_; ++c)
            jointColorStart_[c + 1] += jointColorStart_[c];
        jointFill_ = jointColorStart_;
        jointSorted_.resize(count);
        jointOrder_.resize(count);
        for (int k = 0; k < count; ++k) {
            int sorted = jointFill_[jointColorOf_[k]]++;
            jointSorted_[sorted] = joints[k];
            jointOrder_[sorted] = k;
        }

        size_t bytes = joints.size() * sizeof(Joint);
        if (bytes > jointCap_) {
            release(dJoints_);
            VELOX_CUDA_CHECK(cudaMalloc(&dJoints_, bytes));
            jointCap_ = bytes;
        }
        VELOX_CUDA_CHECK(cudaMemcpy(dJoints_, jointSorted_.data(), bytes,
                                    cudaMemcpyHostToDevice));
        if (!stream_) VELOX_CUDA_CHECK(cudaStreamCreate(&stream_));
        bool reusable = jointGraph_ && jointGraphBodies_ == dBodies_ &&
                        jointGraphJoints_ == dJoints_ && jointGraphDt_ == dt &&
                        previousColorStart == jointColorStart_ &&
                        previousOrder == jointOrder_;
        if (reusable) return;
        if (jointGraph_) {
            cudaGraphExecDestroy(jointGraph_);
            jointGraph_ = nullptr;
        }
        cudaGraph_t graph;
        VELOX_CUDA_CHECK(cudaStreamBeginCapture(stream_,
                                                cudaStreamCaptureModeGlobal));
        for (int iteration = 0; iteration < joint_solver::kIterations;
             ++iteration)
            for (int color = 0; color < jointNumColors_; ++color) {
                int first = jointColorStart_[color];
                int colorCount = jointColorStart_[color + 1] - first;
                solveJointColorKernel<<<(colorCount + 127) / 128, 128, 0,
                                        stream_>>>(dBodies_, dJoints_, first,
                                                   colorCount, dt);
            }
        markBrokenJointsKernel<<<(count + 127) / 128, 128, 0, stream_>>>(
            dJoints_, count, dt);
        VELOX_CUDA_CHECK(cudaStreamEndCapture(stream_, &graph));
        VELOX_CUDA_CHECK(cudaGraphInstantiate(&jointGraph_, graph,
                                              nullptr, nullptr, 0));
        VELOX_CUDA_CHECK(cudaGraphDestroy(graph));
        jointGraphBodies_ = dBodies_;
        jointGraphJoints_ = dJoints_;
        jointGraphDt_ = dt;
    }

    void uploadMeshes(const MeshSoup& m) {
        // Static geometry: upload once, re-upload only if it grew.
        if (m.vertices.size() == meshVerts_ && m.bvhNodes.size() == meshNodes_ &&
            m.hullPoints.size() == hullPts_ &&
            m.compoundChildren.size() == compoundChildren_) return;
        meshVerts_ = m.vertices.size();
        meshNodes_ = m.bvhNodes.size();
        hullPts_ = m.hullPoints.size();
        compoundChildren_ = m.compoundChildren.size();
        upload(dVertices_, m.vertices);
        upload(dIndices_, m.indices);
        upload(dMeshes_, m.meshes);
        upload(dNodes_, m.bvhNodes);
        upload(dTriRefs_, m.bvhTriRefs);
        upload(dHullPts_, m.hullPoints);
        upload(dCompoundChildren_, m.compoundChildren);
    }

    template <typename T>
    void upload(T*& dst, const std::vector<T>& src) {
        release(dst);
        if (src.empty()) return;
        VELOX_CUDA_CHECK(cudaMalloc(&dst, src.size() * sizeof(T)));
        VELOX_CUDA_CHECK(cudaMemcpy(dst, src.data(), src.size() * sizeof(T),
                                    cudaMemcpyHostToDevice));
    }

    static int vmaxContacts(int n) {
        // Manifold-rich scenes (box stacks) can produce many contacts per
        // body; dropping any on overflow injects noise, so size generously.
        int c = n * 24 + 1024;
        return c < 4 << 20 ? c : 4 << 20;
    }

    Body* dBodies_ = nullptr;
    Aabb* dAabbs_ = nullptr;
    float* dSortKeys_ = nullptr;
    BodyIndex* dSortedIndices_ = nullptr;
    uint64_t* dPairs_ = nullptr;
    size_t pairCap_ = 0;
    size_t aabbCap_ = 0;
    Contact* dSolve_ = nullptr;
    size_t solveCap_ = 0;
    int solveCount_ = 0;
    int numColors_ = 0;
    cudaStream_t stream_ = nullptr;
    cudaGraphExec_t solveGraph_ = nullptr;
    cudaGraphExec_t jointGraph_ = nullptr;
    std::vector<int> nextColor_, colorOf_, colorStart_, fill_;
    std::vector<uint64_t> colorMask_;
    std::vector<Contact> sorted_;
    Contact* dContacts_ = nullptr;
    Joint* dJoints_ = nullptr;
    int* dCount_ = nullptr;
    Vec3* dVertices_ = nullptr;
    uint32_t* dIndices_ = nullptr;
    Mesh* dMeshes_ = nullptr;
    BvhNode* dNodes_ = nullptr;
    uint32_t* dTriRefs_ = nullptr;
    Vec3* dHullPts_ = nullptr;
    CompoundChild* dCompoundChildren_ = nullptr;
    size_t hullPts_ = ~size_t(0);
    size_t compoundChildren_ = ~size_t(0);
    size_t bodyCap_ = 0;
    size_t contactCap_ = 0;
    size_t jointCap_ = 0;
    size_t meshVerts_ = ~size_t(0), meshNodes_ = ~size_t(0);
    bool bodiesDirty_ = true;
    int jointNumColors_ = 0;
    std::vector<int> jointNextColor_, jointColorOf_, jointColorStart_,
                     jointFill_, jointOrder_;
    std::vector<uint64_t> jointColorMask_;
    std::vector<Joint> jointSorted_;
    Body* jointGraphBodies_ = nullptr;
    Joint* jointGraphJoints_ = nullptr;
    float jointGraphDt_ = -1.0f;
};

} // namespace

Backend* createCudaBackend() { return CudaBackend::create(); }

} // namespace velox
