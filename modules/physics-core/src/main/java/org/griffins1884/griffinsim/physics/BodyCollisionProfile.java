package org.griffins1884.griffinsim.physics;

public record BodyCollisionProfile(
    String bodyId,
    double halfExtentX,
    double halfExtentY,
    double halfExtentZ,
    MaterialProfile materialProfile) {
  public BodyCollisionProfile {
    if (bodyId == null || bodyId.isBlank()) {
      throw new IllegalArgumentException("bodyId must be present");
    }
    if (halfExtentX <= 0.0 || halfExtentY <= 0.0 || halfExtentZ <= 0.0) {
      throw new IllegalArgumentException("half extents must be positive");
    }
    materialProfile = materialProfile == null ? MaterialProfiles.DEFAULT_BODY : materialProfile;
  }
}
