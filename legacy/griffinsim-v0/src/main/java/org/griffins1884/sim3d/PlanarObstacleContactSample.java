package org.griffins1884.sim3d;

import edu.wpi.first.math.geometry.Translation2d;

/** Planar hard-contact sample against a vertically extruded blocker. */
public record PlanarObstacleContactSample(
    TerrainFeature feature,
    Translation2d outwardNormal,
    double penetrationMeters,
    double obstacleHeightMeters) {}
