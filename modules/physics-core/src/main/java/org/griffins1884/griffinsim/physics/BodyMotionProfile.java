package org.griffins1884.griffinsim.physics;

public record BodyMotionProfile(String bodyId, double massKg, double momentOfInertiaKgM2) {
  public BodyMotionProfile {
    if (bodyId == null || bodyId.isBlank()) {
      throw new IllegalArgumentException("bodyId must be present");
    }
    if (massKg <= 0.0) {
      throw new IllegalArgumentException("massKg must be positive");
    }
    if (momentOfInertiaKgM2 <= 0.0) {
      throw new IllegalArgumentException("momentOfInertiaKgM2 must be positive");
    }
  }
}
