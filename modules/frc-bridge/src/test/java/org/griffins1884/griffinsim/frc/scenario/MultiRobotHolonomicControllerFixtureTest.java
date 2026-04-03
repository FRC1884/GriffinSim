package org.griffins1884.griffinsim.frc.scenario;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.griffins1884.griffinsim.frc.ClockMode;
import org.griffins1884.griffinsim.frc.ControlHostConfig;
import org.griffins1884.griffinsim.physics.DualChannelActuatorCommandMapper;
import org.griffins1884.griffinsim.physics.FieldContactPresets;
import org.griffins1884.griffinsim.physics.ImmutableWorldState;
import org.griffins1884.griffinsim.sensors.SensorEmissionConfig;
import org.junit.jupiter.api.Test;

class MultiRobotHolonomicControllerFixtureTest {
  @Test
  void holonomicControllerFixtureProducesDeterministicPlanarMotion() {
    MultiRobotScenarioRunner runner = new MultiRobotScenarioRunner();
    MultiRobotRunResult first = runner.run(createScenario(), 4);
    MultiRobotRunResult second = runner.run(createScenario(), 4);

    assertArrayEquals(first.replayBytes(), second.replayBytes());
    assertTrue(first.finalWorldState().bodies().get(0).x() != 0.0 || first.finalWorldState().bodies().get(0).y() != 0.0);
    assertEquals(4, first.contactTelemetryFrames().size());
  }

  private static MultiRobotScenario createScenario() {
    ControlHostConfig config = new ControlHostConfig(ClockMode.REAL_TIME, 5_000_000L, 20_000_000L, 16);
    return new MultiRobotScenario(
        "holonomic-controller-fixture",
        config,
        new ImmutableWorldState(
            FrameHeader.current(0L, 0),
            List.of(new RigidBodyState("robot-a", 0.0, 0.0, 0.0, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0))),
        List.of(
            ScenarioEndpointFactory.scheduledHolonomicEndpoint(
                config,
                new ScheduledHolonomicRobotSpec(
                    "robot-a",
                    0,
                    1,
                    PathFollower2dCompiler.compile(
                        new PathFollower2dSpec(
                            List.of(
                                new Waypoint2d(0, 0.0, 0.0, 0.2),
                                new Waypoint2d(1, 1.0, 0.5, 0.8),
                                new Waypoint2d(3, -0.5, -0.25, 0.4)),
                            0.0,
                            0.0,
                            1.2,
                            0.9,
                            0.2,
                            0.01)),
                    1884L,
                    SensorEmissionConfig.immediate("robot-a", 0),
                    new DualChannelActuatorCommandMapper("robot-a", 0, 1, 50.0, 10.0, 100.0, 0.0)))) ,
        FieldContactPresets.rebuilt2026Arena());
  }
}
