#!/usr/bin/env python3
"""Transform world.cpp to use VELOX_THROW macros instead of throw std::xxx."""
import re

with open(r"C:\Users\adria\Desktop\PHYSIC PLACEHOLDER\src\world.cpp", "r", encoding="utf-8") as f:
    content = f.read()

# 1. Replace #include <stdexcept> with #include "velox/error.h"
content = content.replace('#include <stdexcept>', '#include "velox/error.h"')

# 2. Update helper function signatures to accept ErrorCode parameter
# rejectDuplicateHullPoints
content = content.replace(
    'void rejectDuplicateHullPoints(const std::vector<Vec3>& points, const char* message) {',
    'void rejectDuplicateHullPoints(const std::vector<Vec3>& points, ErrorCode code, const char* message) {'
)
content = content.replace(
    '                throw std::invalid_argument(message);\n}\n\nbool validCombineMode',
    '                VELOX_THROW(VeloxInvalidArgument, code, message);\n}\n\nbool validCombineMode'
)

# requireFiniteVec
content = content.replace(
    'void requireFiniteVec(const Vec3& value, const char* message) {\n    if (!finiteVec(value)) throw std::invalid_argument(message);\n}',
    'void requireFiniteVec(const Vec3& value, ErrorCode code, const char* message) {\n    if (!finiteVec(value)) VELOX_THROW(VeloxInvalidArgument, code, message);\n}'
)

# requireMass
content = content.replace(
    'void requireMass(float mass) {\n    if (!finiteFloat(mass) || mass < 0.0f)\n        throw std::invalid_argument("velox: mass must be finite and non-negative");\n}',
    'void requireMass(float mass) {\n    if (!finiteFloat(mass) || mass < 0.0f)\n        VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidMass, "mass must be finite and non-negative");\n}'
)

# requirePositive
content = content.replace(
    'void requirePositive(float value, const char* message) {\n    if (!finiteFloat(value) || value <= 0.0f) throw std::invalid_argument(message);\n}',
    'void requirePositive(float value, ErrorCode code, const char* message) {\n    if (!finiteFloat(value) || value <= 0.0f) VELOX_THROW(VeloxInvalidArgument, code, message);\n}'
)

# requireNonNegative
content = content.replace(
    'void requireNonNegative(float value, const char* message) {\n    if (!finiteFloat(value) || value < 0.0f) throw std::invalid_argument(message);\n}',
    'void requireNonNegative(float value, ErrorCode code, const char* message) {\n    if (!finiteFloat(value) || value < 0.0f) VELOX_THROW(VeloxInvalidArgument, code, message);\n}'
)

# 3. Now replace all direct throw statements (not in helpers)
# These are the throw std::xxx("velox: ...") patterns

replacements = [
    # validateCcdTuning
    ('throw std::invalid_argument("velox: body CCD tuning is invalid");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidConfiguration, "body CCD tuning is invalid");'),
    # validateMultiToiSettings
    ('throw std::invalid_argument("velox: multi-TOI settings are invalid");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidConfiguration, "multi-TOI settings are invalid");'),
    # validateSolverOptions
    ('throw std::invalid_argument("velox: solver options are invalid");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidConfiguration, "solver options are invalid");'),
    # validateRuntimeBody
    ('throw std::invalid_argument("velox: body contains invalid or non-finite state");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidBodyState, "body contains invalid or non-finite state");'),
    ('throw std::invalid_argument("velox: body orientation must be a finite non-zero quaternion");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidOrientation, "body orientation must be a finite non-zero quaternion");'),
    # inertia orientation (multi-line)
    ('throw std::invalid_argument(\n            "velox: inertia orientation must be a finite non-zero quaternion");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidOrientation,\n            "inertia orientation must be a finite non-zero quaternion");'),
    # AccessGuard - step() wrong thread
    ('throw std::logic_error(std::string("velox: step() must be called by the World owner thread; ") +\n                               method + " was called from another thread");',
     'VELOX_THROW(VeloxLogicError, ErrorCode::WrongThread, std::string("step() must be called by the World owner thread; ") +\n                               method + " was called from another thread");'),
    # AccessGuard - cross-thread mutation
    ('throw std::logic_error(std::string("velox: mutation from another thread requires ") +\n                               "ThreadSafetyPolicy::Concurrent (" + method + ")");',
     'VELOX_THROW(VeloxLogicError, ErrorCode::CrossThreadMutation, std::string("mutation from another thread requires ") +\n                               "ThreadSafetyPolicy::Concurrent (" + method + ")");'),
    # AccessGuard - cross-thread query
    ('throw std::logic_error(std::string("velox: cross-thread query requires ") +\n                               "ThreadSafetyPolicy::Relaxed or Concurrent (" + method + ")");',
     'VELOX_THROW(VeloxLogicError, ErrorCode::CrossThreadQuery, std::string("cross-thread query requires ") +\n                               "ThreadSafetyPolicy::Relaxed or Concurrent (" + method + ")");'),
    # resetBackend - CUDA unavailable
    ('throw std::runtime_error("velox: CUDA backend unavailable "\n                                     "(not built with VELOX_ENABLE_CUDA or no device)");',
     'VELOX_THROW(VeloxRuntimeError, ErrorCode::BackendUnavailable,\n                                     "CUDA backend unavailable (not built with VELOX_ENABLE_CUDA or no device)");'),
    # setThreadSafetyPolicy
    ('throw std::invalid_argument("velox: invalid thread safety policy");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidEnumValue, "invalid thread safety policy");'),
    # setDeviceLossPolicy
    ('throw std::invalid_argument("velox: invalid device loss policy");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidEnumValue, "invalid device loss policy");'),
    # setGPUResidentMode
    ('throw std::invalid_argument("velox: invalid GPU resident mode");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidEnumValue, "invalid GPU resident mode");'),
    # setDeterminismMode
    ('throw std::invalid_argument("velox: invalid determinism mode");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidEnumValue, "invalid determinism mode");'),
    # Strict determinism (multi-line)
    ('throw std::logic_error(\n            "velox: Strict determinism requires VELOX_STRICT_FLOATING_POINT=ON at configure time");',
     'VELOX_THROW(VeloxLogicError, ErrorCode::InvalidConfiguration,\n            "Strict determinism requires VELOX_STRICT_FLOATING_POINT=ON at configure time");'),
    # setIslandSolvingMode
    ('throw std::invalid_argument("velox: invalid island solving mode");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidEnumValue, "invalid island solving mode");'),
    ('throw std::logic_error(\n            "velox: Strict determinism requires sequential island solving");',
     'VELOX_THROW(VeloxLogicError, ErrorCode::InvalidConfiguration,\n            "Strict determinism requires sequential island solving");'),
    # resolve BodyId
    ('throw std::out_of_range("velox: invalid or stale body handle");',
     'VELOX_THROW(VeloxOutOfRange, ErrorCode::StaleBodyHandle, "invalid or stale body handle");'),
    # resolve JointId
    ('throw std::out_of_range("velox: invalid or stale joint handle");',
     'VELOX_THROW(VeloxOutOfRange, ErrorCode::StaleJointHandle, "invalid or stale joint handle");'),
    # addBody capacity
    ('throw std::length_error("velox: body capacity exceeded");',
     'VELOX_THROW(VeloxCapacityExceeded, ErrorCode::BodyCapacityExceeded, "body capacity exceeded");'),
    # addJoint capacity
    ('throw std::length_error("velox: joint capacity exceeded");',
     'VELOX_THROW(VeloxCapacityExceeded, ErrorCode::JointCapacityExceeded, "joint capacity exceeded");'),
    # addBox
    ('throw std::invalid_argument("velox: box half extents must be positive");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidShapeGeometry, "box half extents must be positive");'),
    # addRoundedBox
    ('throw std::invalid_argument("velox: rounded box half extents must be positive");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidShapeGeometry, "rounded box half extents must be positive");'),
    # addEllipsoid
    ('throw std::invalid_argument("velox: ellipsoid radii must be positive");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidShapeGeometry, "ellipsoid radii must be positive");'),
    # addConvexHull
    ('throw std::invalid_argument("velox: convex hull requires at least four points");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::HullTooFewPoints, "convex hull requires at least four points");'),
    ('throw std::invalid_argument("velox: convex hull has too many points");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::HullTooManyPoints, "convex hull has too many points");'),
    ('throw std::invalid_argument("velox: convex hull points are coincident");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::HullCoincidentPoints, "convex hull points are coincident");'),
    ('throw std::invalid_argument("velox: convex hull points are collinear");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::HullCollinearPoints, "convex hull points are collinear");'),
    ('throw std::invalid_argument("velox: convex hull points are coplanar");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::HullCoplanarPoints, "convex hull points are coplanar");'),
    ('throw std::length_error("velox: convex hull face capacity exceeded");',
     'VELOX_THROW(VeloxCapacityExceeded, ErrorCode::HullFaceCapacityExceeded, "convex hull face capacity exceeded");'),
    # addCompound
    ('throw std::invalid_argument("velox: compound requires at least one child shape");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::CompoundEmpty, "compound requires at least one child shape");'),
    ('throw std::length_error("velox: compound child capacity exceeded");',
     'VELOX_THROW(VeloxCapacityExceeded, ErrorCode::CompoundChildCapacityExceeded, "compound child capacity exceeded");'),
    ('throw std::invalid_argument("velox: compound child orientation must be finite");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidOrientation, "compound child orientation must be finite");'),
    ('throw std::invalid_argument("velox: compound child orientation must be non-zero");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidOrientation, "compound child orientation must be non-zero");'),
    ('throw std::invalid_argument("velox: compound box half extents must be positive");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidShapeGeometry, "compound box half extents must be positive");'),
    # compound rounded box (multi-line)
    ('throw std::invalid_argument(\n                    "velox: compound rounded box extents must be positive");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidShapeGeometry,\n                    "compound rounded box extents must be positive");'),
    # compound ellipsoid (multi-line)
    ('throw std::invalid_argument(\n                    "velox: compound ellipsoid radii must be positive");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidShapeGeometry,\n                    "compound ellipsoid radii must be positive");'),
    # compound hull
    ('throw std::invalid_argument("velox: compound hull requires at least four points");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::HullTooFewPoints, "compound hull requires at least four points");'),
    ('throw std::length_error("velox: compound hull point capacity exceeded");',
     'VELOX_THROW(VeloxCapacityExceeded, ErrorCode::HullPointCapacityExceeded, "compound hull point capacity exceeded");'),
    ('throw std::invalid_argument("velox: compound hull points are coincident");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::HullCoincidentPoints, "compound hull points are coincident");'),
    ('throw std::invalid_argument("velox: compound hull points are collinear");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::HullCollinearPoints, "compound hull points are collinear");'),
    ('throw std::invalid_argument("velox: compound hull points are coplanar");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::HullCoplanarPoints, "compound hull points are coplanar");'),
    ('throw std::length_error("velox: compound hull face capacity exceeded");',
     'VELOX_THROW(VeloxCapacityExceeded, ErrorCode::HullFaceCapacityExceeded, "compound hull face capacity exceeded");'),
    # compound default case (multi-line)
    ('throw std::invalid_argument(\n                "velox: compound children must be sphere, box, capsule, or hull");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidShapeGeometry,\n                "compound children must be sphere, box, capsule, or hull");'),
    # compound no positive volume
    ('throw std::invalid_argument("velox: compound has no positive volume");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::CompoundNoVolume, "compound has no positive volume");'),
    # addStaticPlane
    ('throw std::invalid_argument("velox: plane normal must be non-zero");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::ZeroVector, "plane normal must be non-zero");'),
    ('throw std::invalid_argument("velox: plane offset must be finite");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::NonFiniteValue, "plane offset must be finite");'),
    # addStaticMesh
    ('throw std::invalid_argument("velox: mesh requires vertices and complete triangles");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::MeshInvalidData, "mesh requires vertices and complete triangles");'),
    ('throw std::invalid_argument("velox: mesh exceeds 32-bit storage limits");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::MeshStorageExceeded, "mesh exceeds 32-bit storage limits");'),
    ('throw std::out_of_range("velox: mesh index is outside the vertex array");',
     'VELOX_THROW(VeloxOutOfRange, ErrorCode::MeshIndexOutOfRange, "mesh index is outside the vertex array");'),
    ('throw std::invalid_argument("velox: mesh contains a degenerate triangle");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::MeshDegenerateTriangle, "mesh contains a degenerate triangle");'),
    # addStaticHeightfield
    ('throw std::invalid_argument("velox: heightfield requires at least 2x2 samples");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::HeightfieldInvalidDims, "heightfield requires at least 2x2 samples");'),
    ('throw std::invalid_argument("velox: heightfield sample count does not match dimensions");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::HeightfieldInvalidDims, "heightfield sample count does not match dimensions");'),
    ('throw std::invalid_argument("velox: heightfield samples must be finite");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::HeightfieldNonFinite, "heightfield samples must be finite");'),
    ('throw std::length_error("velox: heightfield exceeds 32-bit index capacity");',
     'VELOX_THROW(VeloxCapacityExceeded, ErrorCode::HeightfieldCapacityExceeded, "heightfield exceeds 32-bit index capacity");'),
    # setSubsteps
    ('throw std::invalid_argument("velox: substeps must be positive");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::NonPositiveValue, "substeps must be positive");'),
    # setMotionType
    ('throw std::invalid_argument("velox: plane and mesh colliders must remain static");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidMotionType, "plane and mesh colliders must remain static");'),
    ('throw std::invalid_argument("velox: dynamic motion requires positive mass");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidMass, "dynamic motion requires positive mass");'),
    # mutateShape
    ('throw std::invalid_argument("velox: mutated box extents must be positive");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidShapeGeometry, "mutated box extents must be positive");'),
    ('throw std::invalid_argument("velox: mutated rounded box extents must be positive");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidShapeGeometry, "mutated rounded box extents must be positive");'),
    ('throw std::invalid_argument("velox: mutated ellipsoid radii must be positive");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidShapeGeometry, "mutated ellipsoid radii must be positive");'),
    ('throw std::invalid_argument("velox: invalid shape mutation type");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::ShapeMutationInvalid, "invalid shape mutation type");'),
    # scaleShape
    ('throw std::invalid_argument("velox: shape scale must be positive");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::NonPositiveValue, "shape scale must be positive");'),
    ('throw std::invalid_argument("velox: rounded box scaling must be uniform");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::NonUniformScale, "rounded box scaling must be uniform");'),
    ('throw std::invalid_argument("velox: round primitive scaling must be uniform");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::NonUniformScale, "round primitive scaling must be uniform");'),
    ('throw std::invalid_argument("velox: this collider cannot yet be scaled at runtime");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::UnsupportedScale, "this collider cannot yet be scaled at runtime");'),
    # setMassProperties (multi-line throws)
    ('throw std::invalid_argument(\n            "velox: principal inertia must be positive on every axis");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidMass,\n            "principal inertia must be positive on every axis");'),
    ('throw std::invalid_argument(\n            "velox: principal inertia orientation must be finite");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidOrientation,\n            "principal inertia orientation must be finite");'),
    ('throw std::invalid_argument(\n            "velox: principal inertia orientation must be non-zero");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidOrientation,\n            "principal inertia orientation must be non-zero");'),
    ('throw std::invalid_argument(\n            "velox: static-only geometry cannot have mass properties");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidMotionType,\n            "static-only geometry cannot have mass properties");'),
    # setTransform
    ('throw std::invalid_argument("velox: body orientation must be finite");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidOrientation, "body orientation must be finite");'),
    ('throw std::invalid_argument("velox: body orientation must be non-zero");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidOrientation, "body orientation must be non-zero");'),
    ('throw std::invalid_argument("velox: plane and mesh transforms are baked into geometry");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::BakedTransform, "plane and mesh transforms are baked into geometry");'),
    # setLinearVelocity / setAngularVelocity
    ('throw std::invalid_argument("velox: static bodies cannot have velocity");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::StaticBodyMutation, "static bodies cannot have velocity");'),
    # addForce / addForceAtPoint
    ('throw std::invalid_argument("velox: forces require a dynamic body");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::RequiresDynamicBody, "forces require a dynamic body");'),
    # addTorque
    ('throw std::invalid_argument("velox: torque requires a dynamic body");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::RequiresDynamicBody, "torque requires a dynamic body");'),
    # addLinearImpulse / addImpulseAtPoint
    ('throw std::invalid_argument("velox: impulses require a dynamic body");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::RequiresDynamicBody, "impulses require a dynamic body");'),
    # setGravityScale
    ('throw std::invalid_argument("velox: gravity scale must be finite");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::NonFiniteValue, "gravity scale must be finite");'),
    # setLinearDamping
    ('throw std::invalid_argument("velox: linear damping must be finite and non-negative");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::NonFiniteValue, "linear damping must be finite and non-negative");'),
    # setAngularDamping
    ('throw std::invalid_argument("velox: angular damping must be finite and non-negative");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::NonFiniteValue, "angular damping must be finite and non-negative");'),
    # shiftOrigin
    ('throw std::overflow_error("velox: origin shift overflows world coordinates");',
     'VELOX_THROW(VeloxOverflow, ErrorCode::CoordinateOverflow, "origin shift overflows world coordinates");'),
    ('throw std::overflow_error("velox: origin shift overflows plane offset");',
     'VELOX_THROW(VeloxOverflow, ErrorCode::CoordinateOverflow, "origin shift overflows plane offset");'),
    # restoreSnapshot (multi-line)
    ('throw std::invalid_argument(\n            "velox: a snapshot can only be restored to its originating world");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidConfiguration,\n            "a snapshot can only be restored to its originating world");'),
    # Joint same body (appears many times)
    ('throw std::invalid_argument("velox: a joint requires two different bodies");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::JointSameBody, "a joint requires two different bodies");'),
    # Hinge axis
    ('throw std::invalid_argument("velox: hinge axis must be non-zero");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::JointZeroAxis, "hinge axis must be non-zero");'),
    # Cone/twist axis
    ('throw std::invalid_argument("velox: cone/twist axis must be non-zero");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::JointZeroAxis, "cone/twist axis must be non-zero");'),
    # Prismatic axis
    ('throw std::invalid_argument("velox: prismatic-joint axis must be non-zero");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::JointZeroAxis, "prismatic-joint axis must be non-zero");'),
    # Motor joint limits
    ('throw std::invalid_argument("velox: motor joint limits must be non-negative");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::JointInvalidLimits, "motor joint limits must be non-negative");'),
    # hingeAngle
    ('throw std::invalid_argument("velox: hingeAngle requires a hinge joint");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::JointWrongType, "hingeAngle requires a hinge joint");'),
    # coneSwingAngle
    ('throw std::invalid_argument("velox: coneSwingAngle requires a cone/twist joint");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::JointWrongType, "coneSwingAngle requires a cone/twist joint");'),
    # coneTwistAngle
    ('throw std::invalid_argument("velox: coneTwistAngle requires a cone/twist joint");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::JointWrongType, "coneTwistAngle requires a cone/twist joint");'),
    # prismaticTranslation (multi-line)
    ('throw std::invalid_argument(\n            "velox: prismaticTranslation requires a prismatic joint");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::JointWrongType,\n            "prismaticTranslation requires a prismatic joint");'),
    # sixDofLinearTranslation (multi-line)
    ('throw std::invalid_argument(\n            "velox: sixDofLinearTranslation requires a 6DoF joint");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::JointWrongType,\n            "sixDofLinearTranslation requires a 6DoF joint");'),
    # sixDofAngularRotation (multi-line)
    ('throw std::invalid_argument(\n            "velox: sixDofAngularRotation requires a 6DoF joint");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::JointWrongType,\n            "sixDofAngularRotation requires a 6DoF joint");'),
    # queryMultiToi
    ('throw std::invalid_argument("velox: multi-TOI timestep must be finite and positive");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::NonFiniteValue, "multi-TOI timestep must be finite and positive");'),
    # stepImpl
    ('throw std::invalid_argument("velox: timestep must be finite and non-negative");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::NonFiniteValue, "timestep must be finite and non-negative");'),
    ('throw std::invalid_argument("velox: gravity must be finite");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::NonFiniteValue, "gravity must be finite");'),
    # stepImpl body validation
    ('throw std::invalid_argument("velox: box body has invalid half extents");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidShapeGeometry, "box body has invalid half extents");'),
    ('throw std::invalid_argument("velox: rounded box body has invalid half extents");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidShapeGeometry, "rounded box body has invalid half extents");'),
    ('throw std::invalid_argument("velox: ellipsoid body has invalid radii");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidShapeGeometry, "ellipsoid body has invalid radii");'),
    ('throw std::invalid_argument("velox: plane body has invalid geometry");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidShapeGeometry, "plane body has invalid geometry");'),
    ('throw std::out_of_range("velox: mesh body references an invalid mesh");',
     'VELOX_THROW(VeloxOutOfRange, ErrorCode::MeshIndexOutOfRange, "mesh body references an invalid mesh");'),
    ('throw std::out_of_range("velox: hull body references invalid point storage");',
     'VELOX_THROW(VeloxOutOfRange, ErrorCode::StaleBodyHandle, "hull body references invalid point storage");'),
    # compound body references invalid child storage (multi-line)
    ('throw std::out_of_range(\n                    "velox: compound body references invalid child storage");',
     'VELOX_THROW(VeloxOutOfRange, ErrorCode::StaleBodyHandle,\n                    "compound body references invalid child storage");'),
    # compound child contains invalid state (multi-line)
    ('throw std::invalid_argument(\n                        "velox: compound child contains invalid state");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidBodyState,\n                        "compound child contains invalid state");'),
    # joint validation
    ('throw std::invalid_argument("velox: joint contains invalid body endpoints");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidConfiguration, "joint contains invalid body endpoints");'),
    ('throw std::invalid_argument("velox: joint contains an invalid local frame");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::JointZeroAxis, "joint contains an invalid local frame");'),
    ('throw std::invalid_argument("velox: joint contains invalid motor or limit settings");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::JointInvalidLimits, "joint contains invalid motor or limit settings");'),
    # 6DoF settings (multi-line)
    ('throw std::invalid_argument(\n                "velox: joint contains invalid 6DoF settings");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::JointInvalidLimits,\n                "joint contains invalid 6DoF settings");'),
    ('throw std::invalid_argument("velox: joint contains invalid spring settings");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidConfiguration, "joint contains invalid spring settings");'),
    ('throw std::invalid_argument("velox: joint contains invalid break thresholds");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidConfiguration, "joint contains invalid break thresholds");'),
    # contact modifier (multi-line)
    ('throw std::invalid_argument(\n                        "velox: contact modifier returned invalid contact state");',
     'VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidConfiguration,\n                        "contact modifier returned invalid contact state");'),
]

for old, new in replacements:
    count = content.count(old)
    if count == 0:
        print(f"WARNING: not found: {old[:80]}...")
    else:
        content = content.replace(old, new)
        if count > 1:
            print(f"INFO: replaced {count} occurrences of: {old[:80]}...")

# 4. Now update all helper call sites to add ErrorCode and remove "velox: " prefix
# requireFiniteVec calls - all are NonFiniteValue
# Pattern: requireFiniteVec(xxx, "velox: yyy");
content = re.sub(
    r'requireFiniteVec\(([^,]+),\s*"velox: ([^"]+)"\)',
    r'requireFiniteVec(\1, ErrorCode::NonFiniteValue, "\2")',
    content
)

# requirePositive calls - need to determine error code per call
# Shape-related (radius, extents, height) -> InvalidShapeGeometry
# Mass -> InvalidMass
# Other (scale, frequency, cellSize, explosion radius) -> NonPositiveValue
positive_mappings = [
    # Shape geometry related
    (r'requirePositive\(radius, "velox: sphere radius must be finite and positive"\)',
     'requirePositive(radius, ErrorCode::InvalidShapeGeometry, "sphere radius must be finite and positive")'),
    (r'requirePositive\(radius, "velox: capsule radius must be finite and positive"\)',
     'requirePositive(radius, ErrorCode::InvalidShapeGeometry, "capsule radius must be finite and positive")'),
    (r'requirePositive\(halfHeight,\s*"velox: cylinder half height must be finite and positive"\)',
     'requirePositive(halfHeight,\n                    ErrorCode::InvalidShapeGeometry, "cylinder half height must be finite and positive")'),
    (r'requirePositive\(radius, "velox: cylinder radius must be finite and positive"\)',
     'requirePositive(radius, ErrorCode::InvalidShapeGeometry, "cylinder radius must be finite and positive")'),
    (r'requirePositive\(radius, "velox: cone radius must be finite and positive"\)',
     'requirePositive(radius, ErrorCode::InvalidShapeGeometry, "cone radius must be finite and positive")'),
    (r'requirePositive\(height, "velox: cone height must be finite and positive"\)',
     'requirePositive(height, ErrorCode::InvalidShapeGeometry, "cone height must be finite and positive")'),
    (r'requirePositive\(radius, "velox: rounded box radius must be finite and positive"\)',
     'requirePositive(radius, ErrorCode::InvalidShapeGeometry, "rounded box radius must be finite and positive")'),
    # Compound shape geometry
    (r'requirePositive\(shape\.radius,\s*"velox: compound sphere radius must be positive"\)',
     'requirePositive(shape.radius,\n                            ErrorCode::InvalidShapeGeometry, "compound sphere radius must be positive")'),
    (r'requirePositive\(shape\.radius,\s*"velox: compound capsule radius must be positive"\)',
     'requirePositive(shape.radius,\n                            ErrorCode::InvalidShapeGeometry, "compound capsule radius must be positive")'),
    (r'requirePositive\(shape\.radius,\s*"velox: compound cylinder radius must be positive"\)',
     'requirePositive(shape.radius,\n                            ErrorCode::InvalidShapeGeometry, "compound cylinder radius must be positive")'),
    (r'requirePositive\(shape\.capsuleHalfHeight,\s*"velox: compound cylinder half height must be positive"\)',
     'requirePositive(shape.capsuleHalfHeight,\n                            ErrorCode::InvalidShapeGeometry, "compound cylinder half height must be positive")'),
    (r'requirePositive\(shape\.radius,\s*"velox: compound cone radius must be positive"\)',
     'requirePositive(shape.radius,\n                            ErrorCode::InvalidShapeGeometry, "compound cone radius must be positive")'),
    (r'requirePositive\(shape\.capsuleHalfHeight,\s*"velox: compound cone half height must be positive"\)',
     'requirePositive(shape.capsuleHalfHeight,\n                            ErrorCode::InvalidShapeGeometry, "compound cone half height must be positive")'),
    (r'requirePositive\(shape\.radius,\s*"velox: compound rounded box radius must be positive"\)',
     'requirePositive(shape.radius,\n                            ErrorCode::InvalidShapeGeometry, "compound rounded box radius must be positive")'),
    # Mutated shape geometry
    (r'requirePositive\(mutation\.radius, "velox: mutated sphere radius must be positive"\)',
     'requirePositive(mutation.radius, ErrorCode::InvalidShapeGeometry, "mutated sphere radius must be positive")'),
    (r'requirePositive\(mutation\.radius, "velox: mutated primitive radius must be positive"\)',
     'requirePositive(mutation.radius, ErrorCode::InvalidShapeGeometry, "mutated primitive radius must be positive")'),
    (r'requirePositive\(mutation\.capsuleHalfHeight,\s*"velox: mutated primitive half height must be positive"\)',
     'requirePositive(mutation.capsuleHalfHeight,\n                            ErrorCode::InvalidShapeGeometry, "mutated primitive half height must be positive")'),
    (r'requirePositive\(mutation\.radius, "velox: mutated rounded box radius must be positive"\)',
     'requirePositive(mutation.radius, ErrorCode::InvalidShapeGeometry, "mutated rounded box radius must be positive")'),
    # Mass
    (r'requirePositive\(mass, "velox: body mass must be finite and positive"\)',
     'requirePositive(mass, ErrorCode::InvalidMass, "body mass must be finite and positive")'),
    # NonPositiveValue
    (r'requirePositive\(radius, "velox: explosion radius must be positive"\)',
     'requirePositive(radius, ErrorCode::NonPositiveValue, "explosion radius must be positive")'),
    (r'requirePositive\(cellSize, "velox: heightfield cell size must be positive"\)',
     'requirePositive(cellSize, ErrorCode::NonPositiveValue, "heightfield cell size must be positive")'),
    (r'requirePositive\(frequencyHz,\s*"velox: spring frequency must be finite and positive"\)',
     'requirePositive(frequencyHz,\n                    ErrorCode::NonPositiveValue, "spring frequency must be finite and positive")'),
    # stepImpl body validation
    (r'requirePositive\(body\.radius, "velox: sphere body has an invalid radius"\)',
     'requirePositive(body.radius, ErrorCode::InvalidShapeGeometry, "sphere body has an invalid radius")'),
    (r'requirePositive\(body\.radius, "velox: capsule body has an invalid radius"\)',
     'requirePositive(body.radius, ErrorCode::InvalidShapeGeometry, "capsule body has an invalid radius")'),
    (r'requirePositive\(body\.capsuleHalfHeight,\s*"velox: capsule body has an invalid half height"\)',
     'requirePositive(body.capsuleHalfHeight,\n                               ErrorCode::InvalidShapeGeometry, "capsule body has an invalid half height")'),
    (r'requirePositive\(body\.radius, "velox: cylinder body has an invalid radius"\)',
     'requirePositive(body.radius, ErrorCode::InvalidShapeGeometry, "cylinder body has an invalid radius")'),
    (r'requirePositive\(body\.capsuleHalfHeight,\s*"velox: cylinder body has an invalid half height"\)',
     'requirePositive(body.capsuleHalfHeight,\n                            ErrorCode::InvalidShapeGeometry, "cylinder body has an invalid half height")'),
    (r'requirePositive\(body\.radius, "velox: cone body has an invalid radius"\)',
     'requirePositive(body.radius, ErrorCode::InvalidShapeGeometry, "cone body has an invalid radius")'),
    (r'requirePositive\(body\.capsuleHalfHeight,\s*"velox: cone body has an invalid half height"\)',
     'requirePositive(body.capsuleHalfHeight,\n                            ErrorCode::InvalidShapeGeometry, "cone body has an invalid half height")'),
    (r'requirePositive\(body\.radius, "velox: rounded box body has an invalid radius"\)',
     'requirePositive(body.radius, ErrorCode::InvalidShapeGeometry, "rounded box body has an invalid radius")'),
]

for old, new in positive_mappings:
    count = len(re.findall(old, content))
    if count == 0:
        print(f"WARNING requirePositive not found: {old[:80]}...")
    else:
        content = re.sub(old, new, content)

# requireNonNegative calls
nonneg_mappings = [
    (r'requireNonNegative\(halfHeight, "velox: capsule half height must be finite and non-negative"\)',
     'requireNonNegative(halfHeight, ErrorCode::InvalidShapeGeometry, "capsule half height must be finite and non-negative")'),
    (r'requireNonNegative\(shape\.capsuleHalfHeight,\s*"velox: compound capsule half height must be non-negative"\)',
     'requireNonNegative(shape.capsuleHalfHeight,\n                               ErrorCode::InvalidShapeGeometry, "compound capsule half height must be non-negative")'),
    (r'requireNonNegative\(mutation\.capsuleHalfHeight,\s*"velox: mutated capsule half height must be non-negative"\)',
     'requireNonNegative(mutation.capsuleHalfHeight,\n                               ErrorCode::InvalidShapeGeometry, "mutated capsule half height must be non-negative")'),
    (r'requireNonNegative\(margin, "velox: collision margin must be non-negative"\)',
     'requireNonNegative(margin, ErrorCode::NegativeValue, "collision margin must be non-negative")'),
    (r'requireNonNegative\(dampingRatio,\s*"velox: spring damping ratio must be finite and non-negative"\)',
     'requireNonNegative(dampingRatio,\n                       ErrorCode::NegativeValue, "spring damping ratio must be finite and non-negative")'),
    (r'requireNonNegative\(body\.capsuleHalfHeight,\s*"velox: capsule body has an invalid half height"\)',
     'requireNonNegative(body.capsuleHalfHeight,\n                               ErrorCode::InvalidShapeGeometry, "capsule body has an invalid half height")'),
]

for old, new in nonneg_mappings:
    count = len(re.findall(old, content))
    if count == 0:
        print(f"WARNING requireNonNegative not found: {old[:80]}...")
    else:
        content = re.sub(old, new, content)

# rejectDuplicateHullPoints calls
content = content.replace(
    'rejectDuplicateHullPoints(points,\n                              "velox: convex hull contains duplicate points");',
    'rejectDuplicateHullPoints(points, ErrorCode::HullCoincidentPoints,\n                              "convex hull contains duplicate points");'
)
content = content.replace(
    'rejectDuplicateHullPoints(\n                shape.hullPoints, "velox: compound hull contains duplicate points");',
    'rejectDuplicateHullPoints(\n                shape.hullPoints, ErrorCode::HullCoincidentPoints, "compound hull contains duplicate points");'
)

# 5. Handle remaining "velox: " prefixed strings in requireFiniteVec calls that
# might have been missed (multi-line patterns)
# Check for any remaining "velox: " in requireFiniteVec/requirePositive/requireNonNegative calls
# These would be multi-line patterns not caught by the regex above

# Let's check for any remaining "velox: " strings that are NOT in fprintf or comments
# and handle them
remaining_velox = [m for m in re.finditer(r'"velox: ', content)]
for m in remaining_velox:
    # Get context around the match
    start = max(0, m.start() - 100)
    end = min(len(content), m.end() + 100)
    context = content[start:end]
    # Skip fprintf and comment lines
    line_start = content.rfind('\n', 0, m.start()) + 1
    line = content[line_start:content.find('\n', m.start())]
    if 'fprintf' in line or '//' in line.strip()[:2]:
        continue
    print(f"REMAINING velox: at pos {m.start()}: ...{line.strip()[:100]}...")

with open(r"C:\Users\adria\Desktop\PHYSIC PLACEHOLDER\src\world.cpp", "w", encoding="utf-8") as f:
    f.write(content)

print("\nDone! File written.")
