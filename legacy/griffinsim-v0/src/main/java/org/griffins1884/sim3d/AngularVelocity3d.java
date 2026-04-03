package org.griffins1884.sim3d;

/** Angular velocity sample in radians per second around the chassis roll, pitch, and yaw axes. */
public record AngularVelocity3d(
    double rollRateRadPerSec, double pitchRateRadPerSec, double yawRateRadPerSec) {}
