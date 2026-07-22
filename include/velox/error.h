#pragma once

// Velox structured error handling.
//
// Every exception thrown by the engine derives from VeloxException, which
// carries a machine-readable ErrorCode, source-location context (file, line,
// function), and a human-readable recovery suggestion.  Catch VeloxException
// (or std::exception) to handle any engine error; use code() for programmatic
// branching and suggestion() to surface actionable guidance to the user.
//
// Hierarchy (all derive from std::runtime_error via VeloxException):
//
//   VeloxException
//     ├── VeloxInvalidArgument   – bad parameter value
//     ├── VeloxOutOfRange        – stale handle or index outside valid range
//     ├── VeloxCapacityExceeded  – fixed engine limit reached
//     ├── VeloxLogicError        – API misuse / threading violation
//     ├── VeloxRuntimeError      – runtime condition (serialization, backend)
//     └── VeloxOverflow          – arithmetic overflow in engine coordinates

#include <cstdint>
#include <string>
#include <stdexcept>

namespace velox {

// ---------------------------------------------------------------------------
// Error codes – grouped by subsystem for programmatic handling.
// ---------------------------------------------------------------------------
enum class ErrorCode : uint32_t {
    Unknown = 0,

    // --- Input validation (1xx) -------------------------------------------
    NonFiniteValue       = 100,  // NaN or Inf where a finite value is required
    ZeroVector           = 101,  // Zero-length vector / quaternion
    NonPositiveValue     = 102,  // Value must be strictly positive
    NegativeValue        = 103,  // Value must be >= 0
    InvalidEnumValue     = 104,  // Enumerator outside the valid range
    InvalidConfiguration = 105,  // Composite configuration check failed

    // --- Body & shape (2xx) -----------------------------------------------
    InvalidBodyState     = 200,  // Body description contains invalid fields
    InvalidOrientation   = 201,  // Quaternion not finite / zero-length
    InvalidShapeGeometry = 202,  // Shape dimensions invalid (extents, radii…)
    InvalidMass          = 203,  // Mass not finite or negative
    StaleBodyHandle      = 204,  // BodyId no longer refers to a live body
    StaticBodyMutation   = 205,  // Attempted velocity / force on a static body
    RequiresDynamicBody  = 206,  // Force / impulse needs a dynamic body
    InvalidMotionType    = 207,  // Motion-type transition not allowed
    ShapeMutationInvalid = 208,  // Runtime shape mutation parameter invalid
    NonUniformScale      = 209,  // Shape requires uniform scaling
    UnsupportedScale     = 210,  // Collider type cannot be scaled at runtime
    BakedTransform       = 211,  // Plane / mesh transforms are baked

    // --- Convex hull (3xx) ------------------------------------------------
    HullTooFewPoints     = 300,  // Fewer than 4 points
    HullTooManyPoints    = 301,  // More than the engine maximum
    HullCoincidentPoints = 302,  // All points at the same position
    HullCollinearPoints  = 303,  // Points lie on a single line
    HullCoplanarPoints   = 304,  // Points lie on a single plane (zero volume)
    HullNoVolume         = 305,  // Computed hull volume is not positive

    // --- Joint (4xx) ------------------------------------------------------
    JointSameBody        = 400,  // Both joint endpoints reference one body
    JointZeroAxis        = 401,  // Joint axis vector is zero-length
    JointWrongType       = 402,  // Operation requires a different joint type
    StaleJointHandle     = 403,  // JointId no longer refers to a live joint
    JointInvalidLimits   = 404,  // Joint limit values out of valid range

    // --- Query (5xx) ------------------------------------------------------
    InvalidRayCast       = 500,  // Ray origin / direction / distance invalid
    InvalidOverlap       = 501,  // Overlap shape parameters invalid
    InvalidShapeCast     = 502,  // Shape-cast parameters invalid
    InvalidBatchQuery    = 503,  // Batch query type not recognized
    StaleQueryHandle     = 504,  // Async query handle unknown or consumed

    // --- Serialization (6xx) ----------------------------------------------
    SerializedTruncated      = 600,  // Blob ended before all data was read
    SerializedTrailingBytes  = 601,  // Extra bytes after the scene payload
    SerializedBadMagic       = 602,  // Magic number does not match
    SerializedVersionMismatch = 603, // Version newer than this build supports
    SerializedLayoutMismatch = 604,  // Element size differs (struct change)

    // --- Threading (7xx) --------------------------------------------------
    WrongThread          = 700,  // step() called from a non-owner thread
    CrossThreadMutation  = 701,  // Mutation without the required policy
    CrossThreadQuery     = 702,  // Query without the required policy

    // --- Capacity & limits (8xx) ------------------------------------------
    BodyCapacityExceeded        = 800,
    JointCapacityExceeded       = 801,
    HullFaceCapacityExceeded    = 802,
    CompoundChildCapacityExceeded = 803,
    HullPointCapacityExceeded   = 804,
    MeshStorageExceeded         = 805,
    HeightfieldCapacityExceeded = 806,
    ContactCountOverflow        = 807,
    CoordinateOverflow          = 808,

    // --- Mesh & heightfield (9xx) -----------------------------------------
    MeshInvalidData          = 900,  // Vertices / triangles missing or invalid
    MeshIndexOutOfRange      = 901,  // Triangle index exceeds vertex count
    MeshDegenerateTriangle   = 902,  // Triangle has zero area
    HeightfieldInvalidDims   = 903,  // Dimensions < 2×2 or count mismatch
    HeightfieldNonFinite     = 904,  // Height sample is NaN / Inf

    // --- Character & ragdoll (10xx) ---------------------------------------
    CharacterInvalidDesc     = 1000,
    RagdollInvalidBone       = 1001,
    RagdollInvalidGraph      = 1002,
    RagdollCycle             = 1003,
    RagdollDisconnected      = 1004,
    RagdollInvalidMotor      = 1005,
    RagdollUnregisteredRoot  = 1006,

    // --- Compound (11xx) --------------------------------------------------
    CompoundEmpty            = 1100,
    CompoundNoVolume         = 1101,

    // --- Backend (12xx) ---------------------------------------------------
    BackendUnavailable       = 1200,
};

// ---------------------------------------------------------------------------
// errorCodeName – compile-time-friendly name for logging / switch dispatch.
// ---------------------------------------------------------------------------
inline const char* errorCodeName(ErrorCode code) {
    switch (code) {
    case ErrorCode::Unknown:                    return "Unknown";
    case ErrorCode::NonFiniteValue:             return "NonFiniteValue";
    case ErrorCode::ZeroVector:                 return "ZeroVector";
    case ErrorCode::NonPositiveValue:           return "NonPositiveValue";
    case ErrorCode::NegativeValue:              return "NegativeValue";
    case ErrorCode::InvalidEnumValue:           return "InvalidEnumValue";
    case ErrorCode::InvalidConfiguration:       return "InvalidConfiguration";
    case ErrorCode::InvalidBodyState:           return "InvalidBodyState";
    case ErrorCode::InvalidOrientation:         return "InvalidOrientation";
    case ErrorCode::InvalidShapeGeometry:       return "InvalidShapeGeometry";
    case ErrorCode::InvalidMass:                return "InvalidMass";
    case ErrorCode::StaleBodyHandle:            return "StaleBodyHandle";
    case ErrorCode::StaticBodyMutation:         return "StaticBodyMutation";
    case ErrorCode::RequiresDynamicBody:        return "RequiresDynamicBody";
    case ErrorCode::InvalidMotionType:          return "InvalidMotionType";
    case ErrorCode::ShapeMutationInvalid:       return "ShapeMutationInvalid";
    case ErrorCode::NonUniformScale:            return "NonUniformScale";
    case ErrorCode::UnsupportedScale:           return "UnsupportedScale";
    case ErrorCode::BakedTransform:             return "BakedTransform";
    case ErrorCode::HullTooFewPoints:           return "HullTooFewPoints";
    case ErrorCode::HullTooManyPoints:          return "HullTooManyPoints";
    case ErrorCode::HullCoincidentPoints:       return "HullCoincidentPoints";
    case ErrorCode::HullCollinearPoints:        return "HullCollinearPoints";
    case ErrorCode::HullCoplanarPoints:         return "HullCoplanarPoints";
    case ErrorCode::HullNoVolume:               return "HullNoVolume";
    case ErrorCode::JointSameBody:              return "JointSameBody";
    case ErrorCode::JointZeroAxis:              return "JointZeroAxis";
    case ErrorCode::JointWrongType:             return "JointWrongType";
    case ErrorCode::StaleJointHandle:           return "StaleJointHandle";
    case ErrorCode::JointInvalidLimits:         return "JointInvalidLimits";
    case ErrorCode::InvalidRayCast:             return "InvalidRayCast";
    case ErrorCode::InvalidOverlap:             return "InvalidOverlap";
    case ErrorCode::InvalidShapeCast:           return "InvalidShapeCast";
    case ErrorCode::InvalidBatchQuery:          return "InvalidBatchQuery";
    case ErrorCode::StaleQueryHandle:           return "StaleQueryHandle";
    case ErrorCode::SerializedTruncated:        return "SerializedTruncated";
    case ErrorCode::SerializedTrailingBytes:    return "SerializedTrailingBytes";
    case ErrorCode::SerializedBadMagic:         return "SerializedBadMagic";
    case ErrorCode::SerializedVersionMismatch:  return "SerializedVersionMismatch";
    case ErrorCode::SerializedLayoutMismatch:   return "SerializedLayoutMismatch";
    case ErrorCode::WrongThread:                return "WrongThread";
    case ErrorCode::CrossThreadMutation:        return "CrossThreadMutation";
    case ErrorCode::CrossThreadQuery:           return "CrossThreadQuery";
    case ErrorCode::BodyCapacityExceeded:       return "BodyCapacityExceeded";
    case ErrorCode::JointCapacityExceeded:      return "JointCapacityExceeded";
    case ErrorCode::HullFaceCapacityExceeded:   return "HullFaceCapacityExceeded";
    case ErrorCode::CompoundChildCapacityExceeded: return "CompoundChildCapacityExceeded";
    case ErrorCode::HullPointCapacityExceeded:  return "HullPointCapacityExceeded";
    case ErrorCode::MeshStorageExceeded:        return "MeshStorageExceeded";
    case ErrorCode::HeightfieldCapacityExceeded: return "HeightfieldCapacityExceeded";
    case ErrorCode::ContactCountOverflow:       return "ContactCountOverflow";
    case ErrorCode::CoordinateOverflow:         return "CoordinateOverflow";
    case ErrorCode::MeshInvalidData:            return "MeshInvalidData";
    case ErrorCode::MeshIndexOutOfRange:        return "MeshIndexOutOfRange";
    case ErrorCode::MeshDegenerateTriangle:     return "MeshDegenerateTriangle";
    case ErrorCode::HeightfieldInvalidDims:     return "HeightfieldInvalidDims";
    case ErrorCode::HeightfieldNonFinite:       return "HeightfieldNonFinite";
    case ErrorCode::CharacterInvalidDesc:       return "CharacterInvalidDesc";
    case ErrorCode::RagdollInvalidBone:         return "RagdollInvalidBone";
    case ErrorCode::RagdollInvalidGraph:        return "RagdollInvalidGraph";
    case ErrorCode::RagdollCycle:               return "RagdollCycle";
    case ErrorCode::RagdollDisconnected:        return "RagdollDisconnected";
    case ErrorCode::RagdollInvalidMotor:        return "RagdollInvalidMotor";
    case ErrorCode::RagdollUnregisteredRoot:    return "RagdollUnregisteredRoot";
    case ErrorCode::CompoundEmpty:              return "CompoundEmpty";
    case ErrorCode::CompoundNoVolume:           return "CompoundNoVolume";
    case ErrorCode::BackendUnavailable:         return "BackendUnavailable";
    }
    return "Unknown";
}

// ---------------------------------------------------------------------------
// defaultSuggestion – per-code recovery guidance returned when the throw site
// does not supply an explicit hint.
// ---------------------------------------------------------------------------
inline const char* defaultSuggestion(ErrorCode code) {
    switch (code) {
    case ErrorCode::NonFiniteValue:
        return "Ensure all components are finite (not NaN or Inf). "
               "Check for uninitialized variables or division by zero upstream.";
    case ErrorCode::ZeroVector:
        return "Provide a non-zero vector. For orientations, use the identity "
               "quaternion {0, 0, 0, 1} as a safe default.";
    case ErrorCode::NonPositiveValue:
        return "Provide a value strictly greater than zero.";
    case ErrorCode::NegativeValue:
        return "Provide a value greater than or equal to zero.";
    case ErrorCode::InvalidEnumValue:
        return "Check that the enumerator is within the valid range defined "
               "by the corresponding enum class.";
    case ErrorCode::InvalidConfiguration:
        return "Review the composite configuration struct; one or more fields "
               "failed validation. See the message for the specific constraint.";
    case ErrorCode::InvalidBodyState:
        return "Ensure position, orientation, and velocity are all finite "
               "before creating or updating a body.";
    case ErrorCode::InvalidOrientation:
        return "Provide a finite, non-zero quaternion. The identity "
               "quaternion {0, 0, 0, 1} is a safe default.";
    case ErrorCode::InvalidShapeGeometry:
        return "All shape dimensions (half extents, radii, heights) must be "
               "finite and positive.";
    case ErrorCode::InvalidMass:
        return "Mass must be finite and non-negative. Use 0 for static bodies.";
    case ErrorCode::StaleBodyHandle:
        return "The BodyId refers to a body that was already removed. "
               "Re-acquire the handle from addBody / addSphere / etc.";
    case ErrorCode::StaticBodyMutation:
        return "Static bodies cannot have velocity or forces applied. "
               "Change the motion type to Kinematic or Dynamic first.";
    case ErrorCode::RequiresDynamicBody:
        return "Forces, torques, and impulses can only be applied to bodies "
               "with MotionType::Dynamic.";
    case ErrorCode::InvalidMotionType:
        return "Check that the requested motion-type transition is allowed "
               "for this body's collider type.";
    case ErrorCode::ShapeMutationInvalid:
        return "The shape mutation type does not match the body's current "
               "collider. Verify the ShapeType before mutating.";
    case ErrorCode::NonUniformScale:
        return "This shape type only supports uniform scaling. Pass the same "
               "scale factor for x, y, and z.";
    case ErrorCode::UnsupportedScale:
        return "This collider type does not support runtime scaling. "
               "Remove and re-create the body with the desired dimensions.";
    case ErrorCode::BakedTransform:
        return "Plane and mesh collider transforms are baked into their "
               "geometry at creation time and cannot be changed at runtime.";
    case ErrorCode::HullTooFewPoints:
        return "A convex hull requires at least 4 non-degenerate points.";
    case ErrorCode::HullTooManyPoints:
        return "Reduce the point count or simplify the mesh before "
               "creating a convex hull.";
    case ErrorCode::HullCoincidentPoints:
        return "All hull points are at (nearly) the same position. "
               "Provide points that span a 3D volume.";
    case ErrorCode::HullCollinearPoints:
        return "Hull points lie on a single line. Provide points that span "
               "at least a plane, ideally a 3D volume.";
    case ErrorCode::HullCoplanarPoints:
        return "Hull points lie on a single plane, giving zero volume. "
               "Provide points that span a 3D volume.";
    case ErrorCode::HullNoVolume:
        return "The computed convex hull has zero or negative volume. "
               "Check for degenerate or inverted input geometry.";
    case ErrorCode::JointSameBody:
        return "A joint must connect two distinct bodies. Pass different "
               "BodyId values for each endpoint.";
    case ErrorCode::JointZeroAxis:
        return "Provide a non-zero axis vector for the joint.";
    case ErrorCode::JointWrongType:
        return "This operation requires a specific joint type. Check the "
               "JointType before calling type-specific accessors.";
    case ErrorCode::StaleJointHandle:
        return "The JointId refers to a joint that was already removed. "
               "Re-acquire the handle from the add*Joint call.";
    case ErrorCode::JointInvalidLimits:
        return "Joint limit values must be finite and non-negative.";
    case ErrorCode::InvalidRayCast:
        return "Ray origin must be finite, direction must be finite and "
               "non-zero, and maxDist must be non-negative.";
    case ErrorCode::InvalidOverlap:
        return "Overlap shape parameters must be finite with positive "
               "dimensions and a valid orientation.";
    case ErrorCode::InvalidShapeCast:
        return "Shape-cast parameters must be finite with positive "
               "dimensions and a valid orientation.";
    case ErrorCode::InvalidBatchQuery:
        return "Check the batch query type enumerator.";
    case ErrorCode::StaleQueryHandle:
        return "The async query handle is unknown or was already consumed. "
               "Issue a new query to obtain a fresh handle.";
    case ErrorCode::SerializedTruncated:
        return "The serialized blob is incomplete. Verify the data was not "
               "corrupted or truncated during transfer / storage.";
    case ErrorCode::SerializedTrailingBytes:
        return "The blob contains extra bytes after the scene payload. "
               "Verify the data was not concatenated with unrelated content.";
    case ErrorCode::SerializedBadMagic:
        return "The data does not start with the Velox magic number. "
               "Verify this is a Velox scene blob, not another format.";
    case ErrorCode::SerializedVersionMismatch:
        return "The scene was serialized by a newer Velox version. "
               "Upgrade the engine or re-export with a compatible version.";
    case ErrorCode::SerializedLayoutMismatch:
        return "A serialized struct size does not match this build. "
               "Re-serialize the scene with the current engine version.";
    case ErrorCode::WrongThread:
        return "World::step() must be called from the thread that created "
               "the World. Use a command queue or the thread-safe API.";
    case ErrorCode::CrossThreadMutation:
        return "Enable ThreadSafetyPolicy::AllowCrossThreadMutation when "
               "creating the World to permit mutations from other threads.";
    case ErrorCode::CrossThreadQuery:
        return "Enable ThreadSafetyPolicy::AllowCrossThreadQuery when "
               "creating the World to permit queries from other threads.";
    case ErrorCode::BodyCapacityExceeded:
        return "The maximum number of bodies has been reached. Increase "
               "the body capacity in WorldDesc or remove unused bodies.";
    case ErrorCode::JointCapacityExceeded:
        return "The maximum number of joints has been reached. Increase "
               "the joint capacity in WorldDesc or remove unused joints.";
    case ErrorCode::HullFaceCapacityExceeded:
        return "The convex hull generated too many faces. Simplify the "
               "input geometry or increase the face limit.";
    case ErrorCode::CompoundChildCapacityExceeded:
        return "The compound shape has too many children. Reduce the "
               "child count or split into multiple bodies.";
    case ErrorCode::HullPointCapacityExceeded:
        return "Too many convex hull points. Simplify the geometry.";
    case ErrorCode::MeshStorageExceeded:
        return "The mesh exceeds 32-bit vertex/index storage limits. "
               "Split the mesh into smaller pieces.";
    case ErrorCode::HeightfieldCapacityExceeded:
        return "The heightfield exceeds 32-bit index capacity. "
               "Reduce the sample grid dimensions.";
    case ErrorCode::ContactCountOverflow:
        return "The contact buffer overflowed. Reduce scene complexity "
               "or increase the contact capacity.";
    case ErrorCode::CoordinateOverflow:
        return "World coordinates exceeded the representable range. "
               "Use origin shifting more frequently or reduce scene scale.";
    case ErrorCode::MeshInvalidData:
        return "Provide non-empty vertex and index arrays with a triangle "
               "count that is a multiple of 3.";
    case ErrorCode::MeshIndexOutOfRange:
        return "A triangle index references a vertex outside the vertex "
               "array. Validate indices before creating the mesh.";
    case ErrorCode::MeshDegenerateTriangle:
        return "A triangle has zero area (repeated vertices). Remove "
               "degenerate triangles from the mesh data.";
    case ErrorCode::HeightfieldInvalidDims:
        return "Heightfield dimensions must be at least 2x2 and the sample "
               "count must equal width * height.";
    case ErrorCode::HeightfieldNonFinite:
        return "All heightfield samples must be finite (not NaN or Inf).";
    case ErrorCode::CharacterInvalidDesc:
        return "Review CharacterControllerDesc: capsuleRadius must be > 0, "
               "slopeLimitCosine in [0,1], and all values finite.";
    case ErrorCode::RagdollInvalidBone:
        return "Each ragdoll bone must have valid mass properties and a "
               "unique body handle registered with the World.";
    case ErrorCode::RagdollInvalidGraph:
        return "The ragdoll link graph is invalid. A ragdoll with N bones "
               "requires exactly N-1 links forming a tree.";
    case ErrorCode::RagdollCycle:
        return "The ragdoll link graph contains a cycle. Ragdoll links "
               "must form a tree (acyclic connected graph).";
    case ErrorCode::RagdollDisconnected:
        return "The ragdoll link graph is disconnected. Every bone must "
               "be reachable from the root through the link tree.";
    case ErrorCode::RagdollInvalidMotor:
        return "Ragdoll motors require a hinge link and finite, "
               "non-negative torque values.";
    case ErrorCode::RagdollUnregisteredRoot:
        return "The ragdoll root body is not registered with this World. "
               "Create the body in the same World before building the ragdoll.";
    case ErrorCode::CompoundEmpty:
        return "A compound shape requires at least one child shape.";
    case ErrorCode::CompoundNoVolume:
        return "The compound shape has zero total volume. Ensure at least "
               "one child has positive volume.";
    case ErrorCode::BackendUnavailable:
        return "The requested compute backend is not available. Fall back "
               "to BackendType::Cpu or check CUDA installation.";
    default:
        return "Check the error message for details.";
    }
}

// ---------------------------------------------------------------------------
// VeloxException – base class for all engine exceptions.
// ---------------------------------------------------------------------------
class VeloxException : public std::runtime_error {
public:
    VeloxException(ErrorCode code, const char* msg,
                   const char* file, int line, const char* func,
                   const char* suggestion = nullptr)
        : std::runtime_error(buildWhat(code, msg, file, line))
        , code_(code)
        , file_(file)
        , line_(line)
        , function_(func)
        , suggestion_(suggestion ? suggestion : defaultSuggestion(code))
    {}

    VeloxException(ErrorCode code, const std::string& msg,
                   const char* file, int line, const char* func,
                   const char* suggestion = nullptr)
        : VeloxException(code, msg.c_str(), file, line, func, suggestion)
    {}

    ErrorCode   code()       const noexcept { return code_; }
    const char* file()       const noexcept { return file_; }
    int         line()       const noexcept { return line_; }
    const char* function()   const noexcept { return function_; }
    const char* suggestion() const noexcept { return suggestion_.c_str(); }

    // Multi-line diagnostic string including the recovery suggestion.
    std::string detailed() const {
        std::string d = "velox error ";
        d += errorCodeName(code_);
        d += " (";
        d += std::to_string(static_cast<uint32_t>(code_));
        d += "): ";
        d += std::runtime_error::what();
        d += "\n  at ";
        d += file_;
        d += ":";
        d += std::to_string(line_);
        d += " in ";
        d += function_;
        d += "\n  fix: ";
        d += suggestion_;
        return d;
    }

private:
    // Compose the single-line what() string.
    static std::string buildWhat(ErrorCode code, const char* msg,
                                 const char* file, int line) {
        std::string w = "velox: ";
        w += msg;
        w += " [";
        w += errorCodeName(code);
        w += " ";
        w += std::to_string(static_cast<uint32_t>(code));
        w += "] (";
        // Strip directory prefix – keep only the filename.
        const char* base = file;
        for (const char* p = file; *p; ++p)
            if (*p == '/' || *p == '\\') base = p + 1;
        w += base;
        w += ":";
        w += std::to_string(line);
        w += ")";
        return w;
    }

    ErrorCode   code_;
    const char* file_;
    int         line_;
    const char* function_;
    std::string suggestion_;
};

// ---------------------------------------------------------------------------
// Specific exception types – one per std exception they replace.
// ---------------------------------------------------------------------------

/// Bad parameter value (replaces std::invalid_argument).
class VeloxInvalidArgument : public VeloxException {
public:
    using VeloxException::VeloxException;
};

/// Stale handle or index outside valid range (replaces std::out_of_range).
class VeloxOutOfRange : public VeloxException {
public:
    using VeloxException::VeloxException;
};

/// Fixed engine limit reached (replaces std::length_error).
class VeloxCapacityExceeded : public VeloxException {
public:
    using VeloxException::VeloxException;
};

/// API misuse or threading violation (replaces std::logic_error).
class VeloxLogicError : public VeloxException {
public:
    using VeloxException::VeloxException;
};

/// Runtime condition – serialization, backend (replaces std::runtime_error).
class VeloxRuntimeError : public VeloxException {
public:
    using VeloxException::VeloxException;
};

/// Arithmetic overflow in engine coordinates (replaces std::overflow_error).
class VeloxOverflow : public VeloxException {
public:
    using VeloxException::VeloxException;
};

// ---------------------------------------------------------------------------
// Convenience macros – capture source location automatically.
//
//   VELOX_THROW(VeloxInvalidArgument, ErrorCode::NonPositiveValue,
//               "box half extents must be positive");
//
//   VELOX_THROW_HINT(VeloxInvalidArgument, ErrorCode::InvalidMass,
//                    "mass must be finite and non-negative",
//                    "Set mass to 0 for static bodies or a positive value.");
// ---------------------------------------------------------------------------
#define VELOX_THROW(ExType, code, msg)                                        \
    throw ::velox::ExType(code, msg, __FILE__, __LINE__, __func__)

#define VELOX_THROW_HINT(ExType, code, msg, hint)                             \
    throw ::velox::ExType(code, msg, __FILE__, __LINE__, __func__, hint)

} // namespace velox
