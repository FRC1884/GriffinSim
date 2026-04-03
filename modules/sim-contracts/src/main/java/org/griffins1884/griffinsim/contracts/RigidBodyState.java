package org.griffins1884.griffinsim.contracts;

public record RigidBodyState(
    String bodyId,
    double x,
    double y,
    double z,
    double qw,
    double qx,
    double qy,
    double qz,
    double vx,
    double vy,
    double vz,
    double wx,
    double wy,
    double wz) {
  public RigidBodyState {
    if (bodyId == null || bodyId.isBlank()) {
      throw new IllegalArgumentException("bodyId must be present");
    }
  }
}
