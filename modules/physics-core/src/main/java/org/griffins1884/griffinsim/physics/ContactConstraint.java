package org.griffins1884.griffinsim.physics;

public record ContactConstraint(
    String bodyId,
    String contactId,
    double normalX,
    double normalY,
    double normalZ,
    double penetrationMeters,
    double frictionCoefficient,
    double restitutionCoefficient,
    double rollingFrictionCoefficient,
    double torsionalFrictionCoefficient) {
  public ContactConstraint {
    if (bodyId == null || bodyId.isBlank()) {
      throw new IllegalArgumentException("bodyId must be present");
    }
    if (contactId == null || contactId.isBlank()) {
      throw new IllegalArgumentException("contactId must be present");
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

  public ContactConstraint(
      String bodyId,
      String contactId,
      double normalX,
      double normalY,
      double normalZ,
      double penetrationMeters,
      double frictionCoefficient,
      double restitutionCoefficient) {
    this(
        bodyId,
        contactId,
        normalX,
        normalY,
        normalZ,
        penetrationMeters,
        frictionCoefficient,
        restitutionCoefficient,
        0.0,
        0.0);
  }

  public ContactConstraint(
      String bodyId,
      String contactId,
      double normalX,
      double normalY,
      double normalZ,
      double penetrationMeters) {
    this(bodyId, contactId, normalX, normalY, normalZ, penetrationMeters, 0.8, 0.0, 0.0, 0.0);
  }
}
