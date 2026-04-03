package org.griffins1884.griffinsim.physics;

import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;

public record PhysicsStepRequest(
    FrameHeader nextHeader,
    long stepNanos,
    double gravityMps2,
    List<BodyCommand> commands,
    List<ContactConstraint> contacts) {
  public PhysicsStepRequest {
    if (nextHeader == null) {
      throw new NullPointerException("nextHeader");
    }
    if (stepNanos <= 0) {
      throw new IllegalArgumentException("stepNanos must be positive");
    }
    commands = List.copyOf(commands == null ? List.of() : commands);
    contacts = List.copyOf(contacts == null ? List.of() : contacts);
  }

  public static PhysicsStepRequest earthLike(FrameHeader nextHeader, long stepNanos, List<BodyCommand> commands, List<ContactConstraint> contacts) {
    return new PhysicsStepRequest(nextHeader, stepNanos, 9.80665, commands, contacts);
  }
}
