package org.griffins1884.griffinsim.physics;

public record AxisAlignedBoxObstacle(
    String obstacleId,
    double minX,
    double maxX,
    double minY,
    double maxY,
    double minZ,
    double maxZ,
    MaterialProfile materialProfile) {
  public AxisAlignedBoxObstacle {
    if (obstacleId == null || obstacleId.isBlank()) {
      throw new IllegalArgumentException("obstacleId must be present");
    }
    if (!(minX <= maxX && minY <= maxY && minZ <= maxZ)) {
      throw new IllegalArgumentException("box bounds must be ordered");
    }
    materialProfile = materialProfile == null ? MaterialProfiles.DEFAULT_FIELD : materialProfile;
  }

  public AxisAlignedBoxObstacle(
      String obstacleId,
      double minX,
      double maxX,
      double minY,
      double maxY,
      double minZ,
      double maxZ) {
    this(obstacleId, minX, maxX, minY, maxY, minZ, maxZ, MaterialProfiles.DEFAULT_FIELD);
  }
}
