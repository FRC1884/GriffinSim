package org.griffins1884.griffinsim.physics;

public record ContactEvent(
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
  public ContactEvent {
    if (bodyId == null || bodyId.isBlank()) {
      throw new IllegalArgumentException("bodyId must be present");
    }
    if (contactId == null || contactId.isBlank()) {
      throw new IllegalArgumentException("contactId must be present");
    }
  }
}
