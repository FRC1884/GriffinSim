package org.griffins1884.sim3d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Backend-owned 3D chassis snapshot.
 *
 * <p>This becomes the stable state handoff between the underlying simulation engine and higher-level
 * robot integration code.
 */
public record ChassisState3d(
    Pose3d pose,
    Translation3d fieldRelativeLinearVelocityMetersPerSec,
    Translation3d fieldRelativeLinearAccelerationMetersPerSecSq,
    AngularVelocity3d angularVelocityRadPerSec) {}
