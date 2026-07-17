# PROTO18 - Serialization: Review Status

## Status

Implemented on `proto/serialize`: `include/velox/serialization.h` +
`src/serialization.cpp`, verified by `examples/serialize_demo.cpp` (ctest
`velox.serialize`).

## Design

Serialization is built on the proven `WorldSnapshot` machinery: a serialized
scene is the snapshot's complete state encoded as a versioned binary blob —
bodies, handle slots/generations, joints, meshes/hulls/compounds, sleeping
state, contact events, and the persistent-contact warm-start data. Because
restore goes through `World::restoreSnapshot`, a deserialized world does not
merely look the same: it CONTINUES bitwise identically on the CPU backend
(verified for 120 post-restore frames).

- **Blob format**: each snapshot vector is written as (element size, count,
  raw bytes) in a fixed order. Element sizes are validated on load, so a
  struct layout change in a future build rejects old blobs instead of
  misreading them. Trailing/truncated data throws; nothing is partially
  applied (decode fully stages before `restoreSnapshot`).
- **Container** (`packScene`/`unpackScene`): magic `VLOX`, version,
  compression mode, metadata string, blob. Unknown magic, mismatched
  version, and unavailable compression modes all throw.
- **Compression**: `CompressionMode::None` is fully supported;`LZ4`/`ZSTD`
  enum values are reserved for builds that link the external codecs (the
  core library stays dependency-free, per the spec's fallback provision).
- **Replay**: `ReplayRecording` = full initial scene + per-frame dense body
  states. `verifyReplay` re-simulates on a fresh CPU world and compares
  every frame within tolerances, returning the first mismatching frame
  (1-based) or 0 on success.
- `SerializationAccess` (friend of `World`/`WorldSnapshot`) is the single
  point that touches internals; the public API is free functions in
  `velox/serialization.h`.

## Spec deviations

- One opaque state blob instead of separate `bodyData`/`jointData` arrays:
  the spec's per-field format could not capture warm-start contacts, handle
  generations, or sleep state, which resume determinism requires.
- Byte order/layout is platform-native (documented); cross-platform save
  files are future work alongside the compression codecs.
- Replay stores the full initial scene (the spec's recording had no way to
  reconstruct the starting world).

## Verification (`serialize_demo`)

- 62-body scene (boxes, spheres, hull, capsule, plane, ball+hinge joints),
  90 frames in: restore is bitwise identical, and 120 further frames on
  original and restored worlds stay bitwise identical.
- Container pack/unpack preserves metadata and blob (73 KB for the scene).
- A future-version scene and a corrupted magic are both rejected safely.
- A 60-frame replay verifies clean; flipping one exponent bit in frame 30
  is detected exactly at frame 31. (A low mantissa bit under the position
  tolerance is correctly ignored — tolerance-based comparison.)

## Merge recommendation

Ready to merge after normal review.
