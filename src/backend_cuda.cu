// CUDA backend: integration and the full narrow phase (GJK, manifolds, mesh
// BVH traversal) run on the GPU via the same VELOX_HD code the CPU uses.
// Contact and supported joint constraints use graph-colored device passes;
// small or unsupported joint workloads retain the lower-latency CPU path.
#include "narrowphase.h"
#include "joint_solver.h"
#include <cuda_runtime.h>
#include <thrust/device_ptr.h>
#include <thrust/execution_policy.h>
#include <thrust/sort.h>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <stdexcept>

namespace velox {

namespace {

[[noreturn]] void throwCudaError(cudaError_t error, const char* expression,
                                 const char* file, int line) {
    char message[768];
    std::snprintf(message, sizeof(message),
                  "velox CUDA failure: %s (%s) at %s:%d",
                  cudaGetErrorString(error), expression, file, line);
    throw std::runtime_error(message);
}

#define VELOX_CUDA_CHECK(expr)                                                \
    do {                                                                      \
        cudaError_t err_ = (expr);                                            \
        if (err_ != cudaSuccess)                                              \
            throwCudaError(err_, #expr, __FILE__, __LINE__);                 \
    } while (0)

#define VELOX_CUDA_LAUNCH_CHECK() VELOX_CUDA_CHECK(cudaGetLastError())

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
// Planes remain infinite; meshes use their uploaded root bounds.
struct Aabb { Vec3 lo, hi; };

__global__ void aabbKernel(const Body* bodies, const Mesh* meshes, int n,
                           float dt, Aabb* aabbs, float* sortKeys,
                           BodyIndex* sortedIndices, float* maxNormalExtent) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    const Body& b = bodies[i];
    if (b.shape == ShapeType::Plane) {
        aabbs[i] = {{-1e30f, -1e30f, -1e30f}, {1e30f, 1e30f, 1e30f}};
    } else if (b.shape == ShapeType::Mesh) {
        const Mesh& mesh = meshes[b.meshIndex];
        aabbs[i] = {mesh.aabbMin, mesh.aabbMax};
    } else {
        Aabb box;
        bodyAabb(b, dt, box.lo, box.hi);
        aabbs[i] = box;
        // Grid cell size = the largest "normal" body extent. Oversized
        // outliers (giant boxes, meshes, planes) take the brute-force pass
        // instead of bloating every cell. Positive-float atomic max works on
        // the raw int representation.
        float extent = fmaxf(box.hi.x - box.lo.x,
                             fmaxf(box.hi.y - box.lo.y, box.hi.z - box.lo.z));
        if (extent < 1e6f)
            atomicMax((int*)maxNormalExtent, __float_as_int(extent));
    }
    sortKeys[i] = aabbs[i].lo.x;
    sortedIndices[i] = (BodyIndex)i;
}

// Grid keys: cell coordinates of the AABB center packed into 21 bits per
// axis. Packing is exact (no hash collisions), so equal keys mean the same
// cell and a sorted key array supports binary-search neighborhood lookups.
// Oversized bodies (extent > cellSize) get the sentinel key so they sort to
// the end and are enumerated by the brute-force kernel below.
constexpr uint64_t kOversizeKey = ~0ull;

__device__ inline uint64_t packCell(int cx, int cy, int cz) {
    const uint64_t bias = 1u << 20;
    uint64_t x = (uint64_t)(cx + (int)bias) & 0x1fffff;
    uint64_t y = (uint64_t)(cy + (int)bias) & 0x1fffff;
    uint64_t z = (uint64_t)(cz + (int)bias) & 0x1fffff;
    return (x << 42) | (y << 21) | z;
}

__global__ void cellKeyKernel(const Body* bodies, const Aabb* aabbs, int n,
                              float cellSize, uint64_t* keys,
                              BodyIndex* sortedIndices, int* oversize,
                              int* oversizeCount, int oversizeCap) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    sortedIndices[i] = (BodyIndex)i;
    const Aabb& box = aabbs[i];
    float extent = fmaxf(box.hi.x - box.lo.x,
                         fmaxf(box.hi.y - box.lo.y, box.hi.z - box.lo.z));
    if (extent > cellSize || bodies[i].shape == ShapeType::Plane ||
        bodies[i].shape == ShapeType::Mesh) {
        keys[i] = kOversizeKey;
        int slot = atomicAdd(oversizeCount, 1);
        if (slot < oversizeCap) oversize[slot] = i;
        return;
    }
    float inv = 1.0f / cellSize;
    Vec3 center{(box.lo.x + box.hi.x) * 0.5f, (box.lo.y + box.hi.y) * 0.5f,
                (box.lo.z + box.hi.z) * 0.5f};
    keys[i] = packCell((int)floorf(center.x * inv), (int)floorf(center.y * inv),
                       (int)floorf(center.z * inv));
}

// Stage 1a: neighborhood pairs from the sorted grid. Each body binary-searches
// the 27 neighboring cell keys; cells are contiguous runs in the sorted array.
// The atomic count may exceed pairCap; the host grows the buffer and reruns so
// broad-phase overflow never drops candidates.
__global__ void gridPairsKernel(const Body* bodies, const Aabb* aabbs,
                                const uint64_t* sortedKeys,
                                const BodyIndex* sortedIndices, int n,
                                float cellSize, uint64_t* pairs, int* pairCount,
                                int pairCap) {
    int s = blockIdx.x * blockDim.x + threadIdx.x;
    if (s >= n) return;
    uint64_t myKey = sortedKeys[s];
    if (myKey == kOversizeKey) return;
    BodyIndex rawI = sortedIndices[s];
    const Aabb& boxI = aabbs[rawI];
    float inv = 1.0f / cellSize;
    Vec3 center{(boxI.lo.x + boxI.hi.x) * 0.5f, (boxI.lo.y + boxI.hi.y) * 0.5f,
                (boxI.lo.z + boxI.hi.z) * 0.5f};
    int cx = (int)floorf(center.x * inv);
    int cy = (int)floorf(center.y * inv);
    int cz = (int)floorf(center.z * inv);
    for (int dz = -1; dz <= 1; ++dz)
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) {
                uint64_t key = packCell(cx + dx, cy + dy, cz + dz);
                // lower_bound over the sorted keys.
                int lo = 0, hi = n;
                while (lo < hi) {
                    int mid = (lo + hi) >> 1;
                    if (sortedKeys[mid] < key) lo = mid + 1;
                    else hi = mid;
                }
                for (int t = lo; t < n && sortedKeys[t] == key; ++t) {
                    BodyIndex rawJ = sortedIndices[t];
                    if (rawJ <= rawI) continue; // dedupe: symmetric discovery
                    if (!aabbOverlap(boxI.lo, boxI.hi, aabbs[rawJ].lo, aabbs[rawJ].hi))
                        continue;
                    const Body& a = bodies[rawI];
                    const Body& b = bodies[rawJ];
                    if ((a.isStatic() || a.asleep) && (b.isStatic() || b.asleep))
                        continue;
                    if (!a.canCollideWith(b)) continue;
                    int slot = atomicAdd(pairCount, 1);
                    if (slot < pairCap)
                        pairs[slot] = (uint64_t(rawI) << 32) | rawJ;
                }
            }
}

// Stage 1b: oversized bodies (planes, meshes, giant boxes) against everyone.
__global__ void oversizePairsKernel(const Body* bodies, const Aabb* aabbs,
                                    const int* oversize, int oversizeCount,
                                    int n, float cellSize, uint64_t* pairs,
                                    int* pairCount, int pairCap) {
    long long idx = blockIdx.x * (long long)blockDim.x + threadIdx.x;
    if (idx >= (long long)oversizeCount * n) return;
    int o = (int)(idx / n);
    int j = (int)(idx % n);
    int i = oversize[o];
    if (j == i) return;
    // A pair of two oversized bodies is discovered twice; keep the lower's.
    // The oversize criterion must match cellKeyKernel exactly or such pairs
    // duplicate (and duplicate contacts double the solver impulses).
    const Aabb& boxJ = aabbs[j];
    float extentJ = fmaxf(boxJ.hi.x - boxJ.lo.x,
                          fmaxf(boxJ.hi.y - boxJ.lo.y, boxJ.hi.z - boxJ.lo.z));
    bool jOversize = extentJ > cellSize || bodies[j].shape == ShapeType::Plane ||
                     bodies[j].shape == ShapeType::Mesh;
    if (jOversize && j < i) return;
    if (!aabbOverlap(aabbs[i].lo, aabbs[i].hi, boxJ.lo, boxJ.hi)) return;
    const Body& a = bodies[i];
    const Body& b = bodies[j];
    if ((a.isStatic() || a.asleep) && (b.isStatic() || b.asleep)) return;
    if (!a.canCollideWith(b)) return;
    BodyIndex loI = i < j ? (BodyIndex)i : (BodyIndex)j;
    BodyIndex hiI = i < j ? (BodyIndex)j : (BodyIndex)i;
    int slot = atomicAdd(pairCount, 1);
    if (slot < pairCap) pairs[slot] = (uint64_t(loI) << 32) | hiI;
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
        cudaError_t synchronizeError = cudaDeviceSynchronize();
        if (synchronizeError != cudaSuccess)
            std::fprintf(stderr, "velox CUDA cleanup: %s\n",
                         cudaGetErrorString(synchronizeError));
        if (solveGraph_) cudaGraphExecDestroy(solveGraph_);
        if (jointGraph_) cudaGraphExecDestroy(jointGraph_);
        if (stream_) cudaStreamDestroy(stream_);
        release(dBodies_);
        release(dAabbs_);
        release(dSortKeys_);
        release(dSortedIndices_);
        release(dCellKeys_);
        release(dOversize_);
        release(dCellSize_);
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
        release(dHullFaceIndices_);
        release(dCompoundChildren_);
    }

    const char* name() const override { return "cuda"; }

    void integrate(std::vector<Body>& bodies, const Vec3& gravity, float dt) override {
        int n = static_cast<int>(bodies.size());
        if (n == 0) return;
        uploadBodies(bodies);
        integrateKernel<<<(n + 255) / 256, 256>>>(dBodies_, n, gravity, dt);
        VELOX_CUDA_LAUNCH_CHECK();
        VELOX_CUDA_CHECK(cudaMemcpy(bodies.data(), dBodies_, n * sizeof(Body),
                                    cudaMemcpyDeviceToHost));
        bodiesDirty_ = false; // device copy is current
    }

    void findContacts(const std::vector<Body>& bodies, const MeshSoup& meshes,
                      float dt, const std::vector<uint64_t>* /*hostPairs*/,
                      std::vector<Contact>& out) override {
        // The broad phase runs on-device; host candidate pairs are ignored.
        // VELOX_CUDA_PROFILE=1 prints per-stage times (sync-heavy; debug only).
        const bool profile = std::getenv("VELOX_CUDA_PROFILE") != nullptr;
        auto stamp = [&](const char* label, auto&& fn) {
            if (!profile) { fn(); return; }
            cudaDeviceSynchronize();
            auto t0 = std::chrono::steady_clock::now();
            fn();
            cudaDeviceSynchronize();
            auto t1 = std::chrono::steady_clock::now();
            std::fprintf(stderr, "  cuda %-14s %7.3f ms\n", label,
                         std::chrono::duration<double, std::milli>(t1 - t0).count());
        };
        out.clear();
        int n = static_cast<int>(bodies.size());
        if (n < 2) return;
        stamp("uploadBodies", [&] {
            if (bodiesDirty_) uploadBodies(const_cast<std::vector<Body>&>(bodies));
        });
        stamp("uploadMeshes", [&] { uploadMeshes(meshes); });

        int cap = initialContactCapacity(n);
        size_t residentContacts = contactCap_ / sizeof(Contact);
        if (residentContacts > size_t(cap))
            cap = static_cast<int>(residentContacts);
        ensureContactCapacity(cap);
        if (!dCount_) VELOX_CUDA_CHECK(cudaMalloc(&dCount_, sizeof(int)));

        if ((size_t)n * sizeof(Aabb) > aabbCap_) {
            release(dAabbs_);
            release(dSortKeys_);
            release(dSortedIndices_);
            release(dCellKeys_);
            release(dOversize_);
            VELOX_CUDA_CHECK(cudaMalloc(&dAabbs_, n * sizeof(Aabb)));
            VELOX_CUDA_CHECK(cudaMalloc(&dSortKeys_, n * sizeof(float)));
            VELOX_CUDA_CHECK(cudaMalloc(&dSortedIndices_, n * sizeof(BodyIndex)));
            VELOX_CUDA_CHECK(cudaMalloc(&dCellKeys_, n * sizeof(uint64_t)));
            VELOX_CUDA_CHECK(cudaMalloc(&dOversize_, n * sizeof(int)));
            aabbCap_ = n * sizeof(Aabb);
        }
        if (!dCellSize_) VELOX_CUDA_CHECK(cudaMalloc(&dCellSize_, sizeof(float)));
        stamp("aabbKernel", [&] {
            VELOX_CUDA_CHECK(cudaMemset(dCellSize_, 0, sizeof(float)));
            aabbKernel<<<(n + 255) / 256, 256>>>(dBodies_, dMeshes_, n, dt,
                                                 dAabbs_, dSortKeys_,
                                                 dSortedIndices_, dCellSize_);
            VELOX_CUDA_LAUNCH_CHECK();
        });
        MeshSoupView soup{dVertices_, dIndices_, dMeshes_, dNodes_, dTriRefs_,
                          dHullPts_, dHullFaceIndices_, dCompoundChildren_};
        int threads = 256;
        auto collectContacts = [&](auto&& launch) {
            int count = 0;
            for (;;) {
                VELOX_CUDA_CHECK(cudaMemset(dCount_, 0, sizeof(int)));
                launch(cap);
                VELOX_CUDA_CHECK(cudaMemcpy(&count, dCount_, sizeof(int),
                                            cudaMemcpyDeviceToHost));
                if (count < 0)
                    throw std::length_error("velox: CUDA contact count overflow");
                if (count <= cap) break;
                cap = count;
                ensureContactCapacity(cap);
            }
            out.resize(count);
            if (count > 0)
                VELOX_CUDA_CHECK(cudaMemcpy(out.data(), dContacts_,
                                            count * sizeof(Contact),
                                            cudaMemcpyDeviceToHost));
        };
        // The brute-force kernel has excellent occupancy for small scenes;
        // radix sorting and candidate compaction pay off only once quadratic
        // rejection dominates. Keep both and switch at the measured crossover.
        if (n < 4096 || std::getenv("VELOX_CUDA_ALL_PAIRS")) {
            long long pairs = (long long)n * (n - 1) / 2;
            long long blocks = (pairs + threads - 1) / threads;
            collectContacts([&](int outCap) {
                allPairsContactsKernel<<<(unsigned)blocks, threads>>>(
                    dBodies_, dAabbs_, n, soup, dt, dContacts_, dCount_, outCap);
                VELOX_CUDA_LAUNCH_CHECK();
            });
            bodiesDirty_ = true;
            return;
        }

        // Uniform-grid broad phase: cell size = largest normal body extent
        // (read back once, 4 bytes), exact packed cell keys sorted with
        // thrust, then 27-neighborhood lookups plus a brute-force pass for
        // oversized bodies. Replaces the sorted-axis sweep whose per-body
        // windows spanned hundreds of bodies in dense piles (16 ms -> ~1 ms
        // on an 8192-sphere pile).
        float cellSize = 0.0f;
        int oversizeCount = 0;
        stamp("cellKeys", [&] {
            VELOX_CUDA_CHECK(cudaMemcpy(&cellSize, dCellSize_, sizeof(float),
                                        cudaMemcpyDeviceToHost));
            if (cellSize <= 0.0f) cellSize = 1.0f;
            VELOX_CUDA_CHECK(cudaMemset(dCount_, 0, sizeof(int)));
            cellKeyKernel<<<(n + threads - 1) / threads, threads>>>(
                dBodies_, dAabbs_, n, cellSize, dCellKeys_, dSortedIndices_,
                dOversize_, dCount_, n);
            VELOX_CUDA_LAUNCH_CHECK();
            VELOX_CUDA_CHECK(cudaMemcpy(&oversizeCount, dCount_, sizeof(int),
                                        cudaMemcpyDeviceToHost));
        });
        stamp("thrustSort", [&] {
            thrust::device_ptr<uint64_t> keys(dCellKeys_);
            thrust::device_ptr<BodyIndex> indices(dSortedIndices_);
            // Cached scratch allocator: thrust otherwise cudaMallocs and
            // frees temporary storage on every call, which both costs time
            // and produces large frame-to-frame jitter.
            thrust::sort_by_key(thrust::cuda::par(sortAlloc_), keys, keys + n,
                                indices);
        });
        int candidateCap = static_cast<int>(pairCap_ / sizeof(uint64_t));
        if (candidateCap == 0) {
            candidateCap = n * 128 + 1024;
            VELOX_CUDA_CHECK(cudaMalloc(&dPairs_, candidateCap * sizeof(uint64_t)));
            pairCap_ = candidateCap * sizeof(uint64_t);
        }
        int candidateCount = 0;
        auto launchBroadPhase = [&](int cap2) {
            VELOX_CUDA_CHECK(cudaMemset(dCount_, 0, sizeof(int)));
            gridPairsKernel<<<(n + threads - 1) / threads, threads>>>(
                dBodies_, dAabbs_, dCellKeys_, dSortedIndices_, n, cellSize,
                dPairs_, dCount_, cap2);
            VELOX_CUDA_LAUNCH_CHECK();
            if (oversizeCount > 0) {
                long long work = (long long)oversizeCount * n;
                oversizePairsKernel<<<(unsigned)((work + threads - 1) / threads),
                                      threads>>>(
                    dBodies_, dAabbs_, dOversize_, oversizeCount, n, cellSize,
                    dPairs_, dCount_, cap2);
                VELOX_CUDA_LAUNCH_CHECK();
            }
            VELOX_CUDA_CHECK(cudaMemcpy(&candidateCount, dCount_, sizeof(int),
                                        cudaMemcpyDeviceToHost));
        };
        stamp("gridPairs", [&] { launchBroadPhase(candidateCap); });
        if (candidateCount > candidateCap) {
            release(dPairs_);
            candidateCap = candidateCount;
            VELOX_CUDA_CHECK(cudaMalloc(&dPairs_, candidateCap * sizeof(uint64_t)));
            pairCap_ = candidateCap * sizeof(uint64_t);
            launchBroadPhase(candidateCap);
        }
        if (candidateCount > 0) {
            stamp("narrow+copy", [&] {
                collectContacts([&](int outCap) {
                    candidateContactsKernel<<<
                        (candidateCount + threads - 1) / threads, threads>>>(
                        dBodies_, dPairs_, candidateCount, soup, dt,
                        dContacts_, dCount_, outCap);
                    VELOX_CUDA_LAUNCH_CHECK();
                });
            });
        }
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
                VELOX_CUDA_LAUNCH_CHECK();
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
                    VELOX_CUDA_LAUNCH_CHECK();
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
            if (s > 0) VELOX_CUDA_LAUNCH_CHECK();
            if (s > 0 && hasContacts && solveGraph_)
                VELOX_CUDA_CHECK(cudaGraphLaunch(solveGraph_, stream_));
            if (!joints.empty()) {
                int jointCount = static_cast<int>(joints.size());
                resetJointsKernel<<<(jointCount + 127) / 128, 128, 0, stream_>>>(
                    dJoints_, jointCount, s > 0);
                VELOX_CUDA_LAUNCH_CHECK();
                VELOX_CUDA_CHECK(cudaGraphLaunch(jointGraph_, stream_));
            }
            integrateTransformsKernel<<<(n + 255) / 256, 256, 0, stream_>>>(
                dBodies_, n, dt);
            VELOX_CUDA_LAUNCH_CHECK();
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
        meshVerts_ = meshNodes_ = hullPts_ = hullFaceIndices_ =
            compoundChildren_ = (size_t)-1;
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
                VELOX_CUDA_LAUNCH_CHECK();
            }
        markBrokenJointsKernel<<<(count + 127) / 128, 128, 0, stream_>>>(
            dJoints_, count, dt);
        VELOX_CUDA_LAUNCH_CHECK();
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
            m.hullFaceIndices.size() == hullFaceIndices_ &&
            m.compoundChildren.size() == compoundChildren_) return;
        meshVerts_ = m.vertices.size();
        meshNodes_ = m.bvhNodes.size();
        hullPts_ = m.hullPoints.size();
        hullFaceIndices_ = m.hullFaceIndices.size();
        compoundChildren_ = m.compoundChildren.size();
        upload(dVertices_, m.vertices);
        upload(dIndices_, m.indices);
        upload(dMeshes_, m.meshes);
        upload(dNodes_, m.bvhNodes);
        upload(dTriRefs_, m.bvhTriRefs);
        upload(dHullPts_, m.hullPoints);
        upload(dHullFaceIndices_, m.hullFaceIndices);
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

    void ensureContactCapacity(int count) {
        size_t bytes = size_t(count) * sizeof(Contact);
        if (bytes <= contactCap_) return;
        release(dContacts_);
        VELOX_CUDA_CHECK(cudaMalloc(&dContacts_, bytes));
        contactCap_ = bytes;
    }

    static int initialContactCapacity(int n) {
        // Most scenes fit this first allocation. Dense scenes replay after the
        // count pass with an exact larger allocation, so contacts are never
        // truncated by this latency/memory heuristic.
        long long count = (long long)n * 24 + 1024;
        return static_cast<int>(count < (4 << 20) ? count : (4 << 20));
    }

    // Reusable device scratch for thrust (grow-only, freed at teardown).
    struct CachedDeviceAllocator {
        using value_type = char;
        char* buffer = nullptr;
        size_t capacity = 0;
        char* allocate(std::ptrdiff_t bytes) {
            if ((size_t)bytes > capacity) {
                if (buffer) cudaFree(buffer);
                VELOX_CUDA_CHECK(cudaMalloc(&buffer, bytes));
                capacity = (size_t)bytes;
            }
            return buffer;
        }
        void deallocate(char*, size_t) {} // retained for reuse
        ~CachedDeviceAllocator() { if (buffer) cudaFree(buffer); }
    };
    CachedDeviceAllocator sortAlloc_;

    Body* dBodies_ = nullptr;
    Aabb* dAabbs_ = nullptr;
    float* dSortKeys_ = nullptr;
    BodyIndex* dSortedIndices_ = nullptr;
    uint64_t* dCellKeys_ = nullptr;
    int* dOversize_ = nullptr;
    float* dCellSize_ = nullptr;
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
    uint32_t* dHullFaceIndices_ = nullptr;
    CompoundChild* dCompoundChildren_ = nullptr;
    size_t hullPts_ = ~size_t(0);
    size_t hullFaceIndices_ = ~size_t(0);
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
