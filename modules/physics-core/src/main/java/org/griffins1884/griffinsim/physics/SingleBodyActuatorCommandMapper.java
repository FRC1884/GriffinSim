package org.griffins1884.griffinsim.physics;

import java.util.List;
import java.util.Objects;
import org.griffins1884.griffinsim.contracts.ActuatorFrame;
import org.griffins1884.griffinsim.contracts.PwmOutput;

public final class SingleBodyActuatorCommandMapper implements ActuatorCommandMapper {
  private final String bodyId;
  private final double massKg;
  private final double inertiaKgM2;
  private final double forceScale;
  private final double torqueScale;

  public SingleBodyActuatorCommandMapper(
      String bodyId,
      double massKg,
      double inertiaKgM2,
      double forceScale,
      double torqueScale) {
    if (bodyId == null || bodyId.isBlank()) {
      throw new IllegalArgumentException("bodyId must be present");
    }
    if (massKg <= 0.0 || inertiaKgM2 <= 0.0) {
      throw new IllegalArgumentException("mass and inertia must be positive");
    }
    this.bodyId = bodyId;
    this.massKg = massKg;
    this.inertiaKgM2 = inertiaKgM2;
    this.forceScale = forceScale;
    this.torqueScale = torqueScale;
  }

  @Override
  public List<BodyCommand> map(ActuatorFrame frame, ImmutableWorldState currentWorld) {
    Objects.requireNonNull(frame);
    double forceX = 0.0;
    double torqueZ = 0.0;
    for (PwmOutput pwmOutput : frame.pwmOutputs()) {
      forceX += pwmOutput.value() * forceScale;
      torqueZ += (pwmOutput.channel() % 2 == 0 ? 1.0 : -1.0) * pwmOutput.value() * torqueScale;
    }
    return List.of(new BodyCommand(bodyId, massKg, inertiaKgM2, forceX, 0.0, 0.0, 0.0, 0.0, torqueZ));
  }
}
