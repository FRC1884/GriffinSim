package org.griffins1884.griffinsim.physics;

import java.util.List;
import java.util.Objects;
import org.griffins1884.griffinsim.contracts.ActuatorFrame;
import org.griffins1884.griffinsim.contracts.PwmOutput;

public final class DualChannelActuatorCommandMapper implements ActuatorCommandMapper {
  private final String bodyId;
  private final int xChannel;
  private final int yChannel;
  private final double massKg;
  private final double inertiaKgM2;
  private final double forceScale;
  private final double torqueScale;

  public DualChannelActuatorCommandMapper(
      String bodyId,
      int xChannel,
      int yChannel,
      double massKg,
      double inertiaKgM2,
      double forceScale,
      double torqueScale) {
    if (bodyId == null || bodyId.isBlank()) {
      throw new IllegalArgumentException("bodyId must be present");
    }
    if (xChannel < 0 || yChannel < 0) {
      throw new IllegalArgumentException("channels must be non-negative");
    }
    if (massKg <= 0.0 || inertiaKgM2 <= 0.0) {
      throw new IllegalArgumentException("mass and inertia must be positive");
    }
    this.bodyId = bodyId;
    this.xChannel = xChannel;
    this.yChannel = yChannel;
    this.massKg = massKg;
    this.inertiaKgM2 = inertiaKgM2;
    this.forceScale = forceScale;
    this.torqueScale = torqueScale;
  }

  @Override
  public List<BodyCommand> map(ActuatorFrame frame, ImmutableWorldState currentWorld) {
    Objects.requireNonNull(frame);
    double forceX = 0.0;
    double forceY = 0.0;
    for (PwmOutput pwmOutput : frame.pwmOutputs()) {
      if (pwmOutput.channel() == xChannel) {
        forceX += pwmOutput.value() * forceScale;
      } else if (pwmOutput.channel() == yChannel) {
        forceY += pwmOutput.value() * forceScale;
      }
    }
    double torqueZ = (forceX - forceY) * torqueScale;
    return List.of(new BodyCommand(bodyId, massKg, inertiaKgM2, forceX, forceY, 0.0, 0.0, 0.0, torqueZ));
  }
}
