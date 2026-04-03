package org.griffins1884.sim3d;

import edu.wpi.first.math.geometry.Pose3d;

public record TerrainSample(Pose3d pose3d, double rollRadians, double pitchRadians, double heightMeters) {}
