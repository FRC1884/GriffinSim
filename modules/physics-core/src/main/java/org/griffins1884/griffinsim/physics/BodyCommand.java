package org.griffins1884.griffinsim.physics;

public record BodyCommand(
    String bodyId,
    double massKg,
    double momentOfInertiaKgM2,
    double forceX,
    double forceY,
    double forceZ,
    double torqueX,
    double torqueY,
    double torqueZ) {
  public BodyCommand {
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
