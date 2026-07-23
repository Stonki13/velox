// replay_diff_cli: a standalone tool for the deterministic-multiplayer
// debugging workflow this plan targets -- given two recorded ReplayRecording
// traces of the same nominal simulation (e.g. a client's predicted history
// and the server's authoritative one, or the same replay re-run on two
// machines/platforms), report the exact frame, body, and field where they
// first disagree, using rollback.h's findFirstDivergence.
//
// ReplayRecording (serialization.h) has no on-disk format of its own -- it is
// normally produced and consumed in-process via beginReplay/recordReplayFrame.
// This tool adds a minimal, tool-local binary container (not part of the
// public/stable API) so two recordings can be captured on separate runs/
// machines and compared later, offline.
//
// Usage:
//   replay_diff_cli <expected.trace> <actual.trace> [posTol] [velTol] [oriTol]
//   replay_diff_cli --selftest
#include <velox/velox.h>
#include <velox/rollback.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <istream>
#include <ostream>
#include <string>
#include <vector>

using namespace velox;

namespace {

constexpr char kMagic[4] = {'V', 'R', 'P', 'L'};
constexpr uint32_t kTraceFormatVersion = 1;

void writeU32(std::ostream& out, uint32_t v) { out.write(reinterpret_cast<const char*>(&v), sizeof(v)); }
void writeU64(std::ostream& out, uint64_t v) { out.write(reinterpret_cast<const char*>(&v), sizeof(v)); }
void writeFloat(std::ostream& out, float v) { out.write(reinterpret_cast<const char*>(&v), sizeof(v)); }
void writeBytes(std::ostream& out, const std::vector<uint8_t>& bytes) {
    writeU64(out, bytes.size());
    if (!bytes.empty()) out.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
}

bool readU32(std::istream& in, uint32_t& v) { in.read(reinterpret_cast<char*>(&v), sizeof(v)); return bool(in); }
bool readU64(std::istream& in, uint64_t& v) { in.read(reinterpret_cast<char*>(&v), sizeof(v)); return bool(in); }
bool readFloat(std::istream& in, float& v) { in.read(reinterpret_cast<char*>(&v), sizeof(v)); return bool(in); }
bool readBytes(std::istream& in, std::vector<uint8_t>& bytes) {
    uint64_t size = 0;
    if (!readU64(in, size)) return false;
    bytes.resize(size);
    if (size > 0) in.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(size));
    return bool(in) || size == 0;
}

// Saves a ReplayRecording to a simple length-prefixed binary container:
// magic, format version, dt, packed initial scene, then each frame's raw
// bytes. This is a tool-local format for this CLI, not a versioned public
// API -- it exists so two recordings can be diffed after the fact rather
// than only compared in-process.
bool saveTrace(const std::string& path, const ReplayRecording& recording) {
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    out.write(kMagic, sizeof(kMagic));
    writeU32(out, kTraceFormatVersion);
    writeFloat(out, recording.dt);
    writeBytes(out, packScene(recording.initialScene));
    writeU64(out, recording.frames.size());
    for (const auto& frame : recording.frames) writeBytes(out, frame);
    return bool(out);
}

bool loadTrace(const std::string& path, ReplayRecording& recording) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    char magic[4];
    in.read(magic, sizeof(magic));
    if (!in || std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) return false;
    uint32_t version = 0;
    if (!readU32(in, version) || version != kTraceFormatVersion) return false;
    if (!readFloat(in, recording.dt)) return false;
    std::vector<uint8_t> sceneBytes;
    if (!readBytes(in, sceneBytes)) return false;
    recording.initialScene = unpackScene(sceneBytes);
    uint64_t frameCount = 0;
    if (!readU64(in, frameCount)) return false;
    recording.frames.resize(frameCount);
    for (auto& frame : recording.frames)
        if (!readBytes(in, frame)) return false;
    return true;
}

int reportDivergence(const ReplayRecording& expected, const ReplayRecording& actual,
                     float posTol, float velTol, float oriTol) {
    DivergenceReport report = findFirstDivergence(expected, actual, posTol, velTol, oriTol);
    if (!report.diverged) {
        std::printf("replay_diff_cli: no divergence across %zu frames (tolerances: "
                    "pos=%.6g vel=%.6g ori=%.6g)\n",
                    expected.frames.size(), posTol, velTol, oriTol);
        return 0;
    }
    std::printf("replay_diff_cli: first divergence at frame=%llu body=%u field=%s magnitude=%.6g\n",
               static_cast<unsigned long long>(report.frame), report.bodyIndex,
               report.field.c_str(), report.magnitude);
    return 1;
}

int selftest() {
    World world(BackendType::Cpu);
    world.gravity = {0, -9.81f, 0};
    world.addStaticPlane({0, 1, 0}, 0.0f);
    world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    ReplayRecording expected;
    beginReplay(expected, world, 1.0f / 60.0f);
    for (int i = 0; i < 30; ++i) {
        world.step(1.0f / 60.0f);
        recordReplayFrame(expected, world);
    }

    // "actual" starts identical, then diverges at frame 15 by corrupting one
    // body's stored position -- deliberately, to prove the round-tripped
    // save/load + diff pipeline finds it at the expected spot.
    ReplayRecording actual = expected;
    if (actual.frames.size() < 20) {
        std::fprintf(stderr, "replay_diff_cli selftest: not enough frames recorded\n");
        return 1;
    }
    // Corrupt body 0's position.x (the record's first 4 bytes) at frame 15.
    float px = 0.0f;
    std::memcpy(&px, actual.frames[14].data(), sizeof(px));
    px += 5.0f;
    std::memcpy(actual.frames[14].data(), &px, sizeof(px));

    const std::string expectedPath = "replay_diff_selftest_expected.trace";
    const std::string actualPath = "replay_diff_selftest_actual.trace";
    if (!saveTrace(expectedPath, expected) || !saveTrace(actualPath, actual)) {
        std::fprintf(stderr, "replay_diff_cli selftest: failed to save trace files\n");
        return 1;
    }

    ReplayRecording loadedExpected, loadedActual;
    if (!loadTrace(expectedPath, loadedExpected) || !loadTrace(actualPath, loadedActual)) {
        std::fprintf(stderr, "replay_diff_cli selftest: failed to load trace files\n");
        return 1;
    }

    DivergenceReport report = findFirstDivergence(loadedExpected, loadedActual);
    bool ok = report.diverged && report.frame == 15 && report.bodyIndex == 0 &&
             report.field == "position";
    std::printf("replay_diff_cli selftest: %s (frame=%llu body=%u field=%s)\n",
               ok ? "PASS" : "FAIL", static_cast<unsigned long long>(report.frame),
               report.bodyIndex, report.field.c_str());
    return ok ? 0 : 1;
}

} // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--selftest") return selftest();

    if (argc < 3) {
        std::fprintf(stderr,
                     "usage: replay_diff_cli <expected.trace> <actual.trace> "
                     "[posTol] [velTol] [oriTol]\n"
                     "       replay_diff_cli --selftest\n");
        return 2;
    }

    ReplayRecording expected, actual;
    if (!loadTrace(argv[1], expected)) {
        std::fprintf(stderr, "replay_diff_cli: failed to load '%s'\n", argv[1]);
        return 2;
    }
    if (!loadTrace(argv[2], actual)) {
        std::fprintf(stderr, "replay_diff_cli: failed to load '%s'\n", argv[2]);
        return 2;
    }

    float posTol = argc > 3 ? std::atof(argv[3]) : 1e-5f;
    float velTol = argc > 4 ? std::atof(argv[4]) : 1e-4f;
    float oriTol = argc > 5 ? std::atof(argv[5]) : 1e-5f;
    return reportDivergence(expected, actual, posTol, velTol, oriTol);
}
