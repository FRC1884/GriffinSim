package org.griffins1884.griffinsim.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import org.griffins1884.griffinsim.contracts.ActuatorFrame;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.PwmOutput;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.junit.jupiter.api.Test;

class SingleBodyActuatorCommandMapperTest {
  @Test
  void mapsPwmOutputsToDeterministicForceAndTorque() {
    SingleBodyActuatorCommandMapper mapper = new SingleBodyActuatorCommandMapper("robot", 50.0, 10.0, 100.0, 20.0);
    ActuatorFrame frame =
        new ActuatorFrame(FrameHeader.current(20_000_000L, 1), List.of(new PwmOutput(0, 0.5), new PwmOutput(1, -0.25)));
    ImmutableWorldState world =
        new ImmutableWorldState(FrameHeader.current(0L, 0), List.of(new RigidBodyState("robot", 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)));

    BodyCommand command = mapper.map(frame, world).get(0);

    assertEquals(25.0, command.forceX(), 1e-9);
    assertEquals(15.0, command.torqueZ(), 1e-9);
  }
}
