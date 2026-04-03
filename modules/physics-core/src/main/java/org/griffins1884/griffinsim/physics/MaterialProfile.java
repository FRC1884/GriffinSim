package org.griffins1884.griffinsim.physics;

public record MaterialProfile(
    String name,
    double frictionCoefficient,
    double restitutionCoefficient,
    double rollingFrictionCoefficient,
    double torsionalFrictionCoefficient) {
  public MaterialProfile {
    if (name == null || name.isBlank()) {
      throw new IllegalArgumentException("name must be present");
    }
    if (frictionCoefficient < 0.0) {
      throw new IllegalArgumentException("frictionCoefficient must be non-negative");
    }
    if (restitutionCoefficient < 0.0 || restitutionCoefficient > 1.0) {
      throw new IllegalArgumentException("restitutionCoefficient must be between 0 and 1");
    }
    if (rollingFrictionCoefficient < 0.0) {
      throw new IllegalArgumentException("rollingFrictionCoefficient must be non-negative");
    }
    if (torsionalFrictionCoefficient < 0.0) {
      throw new IllegalArgumentException("torsionalFrictionCoefficient must be non-negative");
    }
  }

  public static MaterialProfile of(
      String name,
      double frictionCoefficient,
      double restitutionCoefficient,
      double rollingFrictionCoefficient,
      double torsionalFrictionCoefficient) {
    return new MaterialProfile(
        name,
        frictionCoefficient,
        restitutionCoefficient,
        rollingFrictionCoefficient,
        torsionalFrictionCoefficient);
  }
}
