package org.griffins1884.griffinsim.contracts;

public record ContactTelemetry(
    String bodyId,
    String contactId,
    double penetrationMeters,
    double normalSpeedBefore,
    double tangentialSpeedBefore,
    double tangentialSpeedAfter,
    double slipRatio,
    double frictionCoefficient,
    double restitutionCoefficient,
    double rollingFrictionCoefficient,
    double torsionalFrictionCoefficient) {
  public ContactTelemetry {
    if (bodyId == null || bodyId.isBlank()) {
      throw new IllegalArgumentException("bodyId must be present");
    }
    if (contactId == null || contactId.isBlank()) {
      throw new IllegalArgumentException("contactId must be present");
    }
  }
}
