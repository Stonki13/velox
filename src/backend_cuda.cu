// CUDA backend: integration and the full narrow phase (GJK, manifolds, mesh
// BVH traversal) run on the GPU via the same VELOX_HD code the CPU uses.
// The contact solver stays on the CPU for now (sequential impulses are
// inherently serial; a Jacobi/graph-colored GPU solver is on the roadmap).
#include "narrowphase.h"
#include <cuda_runtime.h>
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
    if (b.isStatic()) return;
    b.velocity += gravity * dt;
}

// Compact AABBs so the pair kernel can reject without touching Body structs.
// Planes and meshes get infinite bounds (they are collided by every pair).
struct Aabb { Vec3 lo, hi; };

__global__ void aabbKernel(const Body* bodies, int n, float dt, Aabb* aabbs) {
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
}

// One thread per body pair (upper triangle, linearized). The AABB test hits
// only the compact array (fits in L2); full bodies load after an overlap.
__global__ void contactsKernel(const Body* bodies, const Aabb* aabbs, int n,
                               MeshSoupView soup, float dt,
                               Contact* out, int* outCount, int outCap) {
    long long idx = blockIdx.x * (long long)blockDim.x + threadIdx.x;
    long long total = (long long)n * (n - 1) / 2;
    if (idx >= total) return;

    // Linear index -> (i, j), j > i, over the upper triangle.
    int i = (int)((2LL * n - 1 - sqrtf((float)((2LL * n - 1) * (2LL * n - 1) - 8LL * idx))) / 2);
    long long rowStart = (long long)i * n - (long long)i * (i + 1) / 2;
    int j = (int)(idx - rowStart + i + 1);
    // Float sqrt can land one row off; correct it.
    while (j >= n) { ++i; rowStart = (long long)i * n - (long long)i * (i + 1) / 2; j = (int)(idx - rowStart + i + 1); }

    if (!aabbOverlap(aabbs[i].lo, aabbs[i].hi, aabbs[j].lo, aabbs[j].hi)) return;

    const Body& a = bodies[i];
    const Body& b = bodies[j];
    if (a.isStatic() && b.isStatic()) return;

    Contact buf[kMaxContactsPerPair];
    int count = collidePair(a, b, (BodyId)i, (BodyId)j, soup, dt,
                            buf, kMaxContactsPerPair);
    if (count == 0) return;
    int slot = atomicAdd(outCount, count);
    for (int c = 0; c < count && slot + c < outCap; ++c)
        out[slot + c] = buf[c];
}

// Solves every contact of one color in parallel: within a color no two
// contacts share a dynamic body, so the writes cannot conflict.
__global__ void solveColorKernel(Body* bodies, Contact* contacts,
                                 int first, int count, float dt) {
    int k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k >= count) return;
    Contact& c = contacts[first + k];
    solveContact(bodies[c.a], bodies[c.b], c, dt);
}

class CudaBackend final : public Backend {
public:
    static CudaBackend* create() {
        int count = 0;
        if (cudaGetDeviceCount(&count) != cudaSuccess || count == 0) return nullptr;
        return new CudaBackend();
    }

    ~CudaBackend() override {
        release(dBodies_);
        release(dAabbs_);
        release(dSolve_);
        release(dContacts_);
        release(dCount_);
        release(dVertices_);
        release(dIndices_);
        release(dMeshes_);
        release(dNodes_);
        release(dTriRefs_);
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
            VELOX_CUDA_CHECK(cudaMalloc(&dAabbs_, n * sizeof(Aabb)));
            aabbCap_ = n * sizeof(Aabb);
        }
        aabbKernel<<<(n + 255) / 256, 256>>>(dBodies_, n, dt, dAabbs_);

        MeshSoupView soup{dVertices_, dIndices_, dMeshes_, dNodes_, dTriRefs_};
        long long pairs = (long long)n * (n - 1) / 2;
        int threads = 256;
        long long blocks = (pairs + threads - 1) / threads;
        contactsKernel<<<(unsigned)blocks, threads>>>(dBodies_, dAabbs_, n, soup, dt,
                                                      dContacts_, dCount_, cap);
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
                         std::vector<Contact>& contacts, float dt) override {
        int n = static_cast<int>(bodies.size());
        int m = static_cast<int>(contacts.size());
        if (m == 0) return;

        // Greedy coloring on the host: a contact's color is one past the
        // highest color already used by either dynamic endpoint, so no color
        // ever contains two contacts sharing a dynamic body. Static bodies
        // are never written by the solver and don't constrain colors.
        nextColor_.assign(n, 0);
        colorOf_.resize(m);
        int numColors = 0;
        for (int k = 0; k < m; ++k) {
            const Contact& c = contacts[k];
            int ca = bodies[c.a].isStatic() ? 0 : nextColor_[c.a];
            int cb = bodies[c.b].isStatic() ? 0 : nextColor_[c.b];
            int color = ca > cb ? ca : cb;
            colorOf_[k] = color;
            if (!bodies[c.a].isStatic()) nextColor_[c.a] = color + 1;
            if (!bodies[c.b].isStatic()) nextColor_[c.b] = color + 1;
            if (color + 1 > numColors) numColors = color + 1;
        }

        // Counting sort contacts into color order.
        colorStart_.assign(numColors + 1, 0);
        for (int k = 0; k < m; ++k) ++colorStart_[colorOf_[k] + 1];
        for (int c = 0; c < numColors; ++c) colorStart_[c + 1] += colorStart_[c];
        sorted_.resize(m);
        fill_ = colorStart_;
        for (int k = 0; k < m; ++k) sorted_[fill_[colorOf_[k]]++] = contacts[k];

        if (bodiesDirty_) uploadBodies(bodies);
        if ((size_t)m * sizeof(Contact) > solveCap_) {
            release(dSolve_);
            VELOX_CUDA_CHECK(cudaMalloc(&dSolve_, m * sizeof(Contact)));
            solveCap_ = m * sizeof(Contact);
        }
        VELOX_CUDA_CHECK(cudaMemcpy(dSolve_, sorted_.data(), m * sizeof(Contact),
                                    cudaMemcpyHostToDevice));

        for (int iter = 0; iter < kVelocityIterations; ++iter)
            for (int c = 0; c < numColors; ++c) {
                int first = colorStart_[c], count = colorStart_[c + 1] - first;
                solveColorKernel<<<(count + 127) / 128, 128>>>(dBodies_, dSolve_,
                                                               first, count, dt);
            }

        VELOX_CUDA_CHECK(cudaMemcpy(bodies.data(), dBodies_, n * sizeof(Body),
                                    cudaMemcpyDeviceToHost));
        bodiesDirty_ = false;
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

    void uploadMeshes(const MeshSoup& m) {
        // Static geometry: upload once, re-upload only if it grew.
        if (m.vertices.size() == meshVerts_ && m.bvhNodes.size() == meshNodes_) return;
        meshVerts_ = m.vertices.size();
        meshNodes_ = m.bvhNodes.size();
        upload(dVertices_, m.vertices);
        upload(dIndices_, m.indices);
        upload(dMeshes_, m.meshes);
        upload(dNodes_, m.bvhNodes);
        upload(dTriRefs_, m.bvhTriRefs);
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
        int c = n * 8;
        return c < 1 << 20 ? c : 1 << 20;
    }

    Body* dBodies_ = nullptr;
    Aabb* dAabbs_ = nullptr;
    size_t aabbCap_ = 0;
    Contact* dSolve_ = nullptr;
    size_t solveCap_ = 0;
    std::vector<int> nextColor_, colorOf_, colorStart_, fill_;
    std::vector<Contact> sorted_;
    Contact* dContacts_ = nullptr;
    int* dCount_ = nullptr;
    Vec3* dVertices_ = nullptr;
    uint32_t* dIndices_ = nullptr;
    Mesh* dMeshes_ = nullptr;
    BvhNode* dNodes_ = nullptr;
    uint32_t* dTriRefs_ = nullptr;
    size_t bodyCap_ = 0;
    size_t contactCap_ = 0;
    size_t meshVerts_ = ~size_t(0), meshNodes_ = ~size_t(0);
    bool bodiesDirty_ = true;
};

} // namespace

Backend* createCudaBackend() { return CudaBackend::create(); }

} // namespace velox
