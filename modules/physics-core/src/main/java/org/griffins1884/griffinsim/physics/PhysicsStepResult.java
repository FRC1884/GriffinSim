package org.griffins1884.griffinsim.physics;

import java.util.List;

public record PhysicsStepResult(ImmutableWorldState worldState, List<ContactEvent> contactEvents) {
  public PhysicsStepResult {
    if (worldState == null) {
      throw new NullPointerException("worldState");
    }
    contactEvents = List.copyOf(contactEvents == null ? List.of() : contactEvents);
  }
}
