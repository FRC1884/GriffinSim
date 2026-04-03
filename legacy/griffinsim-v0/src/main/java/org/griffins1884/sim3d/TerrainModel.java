package org.griffins1884.sim3d;

import edu.wpi.first.math.geometry.Pose2d;

public interface TerrainModel {
  TerrainSample sample(Pose2d robotPose);
}
