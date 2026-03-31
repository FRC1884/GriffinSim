package org.griffins1884.sim3d;

/** Per-wheel normal load and derived traction capacity estimate. */
public record WheelLoadSample(
    double normalForceNewtons, double tractionCapacityNewtons, double normalizedLoad) {}
