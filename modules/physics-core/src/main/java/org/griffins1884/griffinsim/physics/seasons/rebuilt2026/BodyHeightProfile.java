package org.griffins1884.griffinsim.physics.seasons.rebuilt2026;

public record BodyHeightProfile(String bodyId, double heightMeters) {
  public BodyHeightProfile {
    if (bodyId == null || bodyId.isBlank()) {
      throw new IllegalArgumentException("bodyId must be present");
    }
    if (heightMeters <= 0.0) {
      throw new IllegalArgumentException("heightMeters must be positive");
    }
  }
}
