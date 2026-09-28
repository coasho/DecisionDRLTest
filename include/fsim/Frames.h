#pragma once

// Reference frames (docs/flight-autonomy.md, 4.21; A-GRA's ReferenceFrame and its relative points): a frame by id,
// fixed, moving at a constant velocity, or following a vehicle; a point in it - offsets turned as A-GRA's RotationEnum
// says and laid out on the Earth as its OffsetXY_Enum says - as a place at a time. The Earth is a sphere of the mean
// radius, as for all the platform's local geometry (core/Geodesy.h); heights are above sea level.

#include "fsim/Export.h"

#include <cstdint>

namespace fsim::control {

/// A frame's origin (A-GRA's ReferenceFrameOriginChoiceType).
enum class FrameOrigin : std::uint8_t {
    Fixed = 0,   ///< a place (GeospatialPosition)
    Moving = 1,  ///< a place moving at a constant velocity from a time (KinematicsReferenceFrameOrigin)
    Vehicle = 2, ///< a world vehicle, followed (ObjectToFollowIdentifier)
    Count
};

/// How a point's offsets are turned (A-GRA's RotationEnum).
enum class FrameRotation : std::uint8_t {
    Unrotated = 0, ///< x north, y east, z down
    Yaw = 1,       ///< turned by the origin's yaw (ROTATION_2D)
    Attitude = 2,  ///< its body's axes: x forward, y right, z down, as it is pitched and rolled too (ROTATION_3D)
    Heading = 3,   ///< turned by its track over the ground (HEADING); its yaw where it has none
    Count
};

/// How they are laid out on the Earth (A-GRA's OffsetXY_Enum).
enum class FrameOffsets : std::uint8_t {
    Cartesian = 0,   ///< in the plane square to the vertical at the origin
    GreatCircle = 1, ///< as far as x and y make, along the great circle their way
    Rhumb = 2,       ///< along the rhumb line their way
    Count
};

/// A reference frame's id: 0 is none.
using FrameId = std::uint64_t;

/// A frame (A-GRA's ReferenceFrame).
struct FrameSpec {
    FrameOrigin origin = FrameOrigin::Fixed;
    double latitudeRad = 0.0, longitudeRad = 0.0, altitudeMslM = 0.0; ///< a fixed origin's; a moving one's at `timeS`
    double yawRad = 0.0, pitchRad = 0.0, rollRad = 0.0;               ///< a fixed or moving origin's orientation
    double northMs = 0.0, eastMs = 0.0, downMs = 0.0;                 ///< a moving origin's velocity
    double timeS = 0.0;                                               ///< when a moving origin is at its place (World::time)
    std::uint32_t vehicle = 0;                                        ///< the vehicle a frame follows
};

/// A point in a frame (A-GRA's RelativeOffset).
struct FrameOffset {
    FrameRotation rotation = FrameRotation::Unrotated;
    FrameOffsets offsets = FrameOffsets::Cartesian;
    double x = 0.0, y = 0.0, z = 0.0; ///< m along the axes the rotation gives; z down
};

/// A frame as it is at a time: its origin, orientation and velocity.
struct FramePose {
    double latitudeRad = 0.0, longitudeRad = 0.0, altitudeMslM = 0.0;
    double yawRad = 0.0, pitchRad = 0.0, rollRad = 0.0;
    double northMs = 0.0, eastMs = 0.0, downMs = 0.0;
};

/// A place: above sea level.
struct GeoPoint {
    double latitudeRad = 0.0, longitudeRad = 0.0, altitudeMslM = 0.0;
};

/// A fixed or moving frame's pose at a time: a moving origin carried on at its velocity.
FSIM_API FramePose framePose(const FrameSpec& spec, double timeS) noexcept;
/// A pose carried on at its velocity for `seconds` (a vehicle's, to another time).
FSIM_API FramePose carried(const FramePose& pose, double seconds) noexcept;
/// Where a point in a frame is, the frame as it is.
FSIM_API GeoPoint framePoint(const FramePose& pose, const FrameOffset& offset) noexcept;

} // namespace fsim::control
