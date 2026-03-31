package org.griffins1884.sim3d;

import edu.wpi.first.math.geometry.Rotation3d;

/** Simulated IMU orientation and angular-rate sample. */
public record SimImuSample(
    Rotation3d orientation,
    double yawRateRadPerSec,
    double pitchRateRadPerSec,
    double rollRateRadPerSec) {}
