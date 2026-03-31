package org.griffins1884.sim3d;

import edu.wpi.first.math.geometry.Pose2d;

/** Terrain model with explicit clearance/contact semantics for a robot footprint. */
public interface TerrainContactModel extends TerrainModel {
  TerrainContactSample sampleContact(Pose2d robotPose, ChassisFootprint chassisFootprint);
}
