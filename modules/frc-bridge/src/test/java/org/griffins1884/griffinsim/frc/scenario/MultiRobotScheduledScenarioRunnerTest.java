package org.griffins1884.griffinsim.frc.scenario;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.griffins1884.griffinsim.frc.ClockMode;
import org.griffins1884.griffinsim.frc.ControlHostConfig;
import org.griffins1884.griffinsim.physics.FieldContactPresets;
import org.griffins1884.griffinsim.physics.ImmutableWorldState;
import org.griffins1884.griffinsim.physics.SingleBodyActuatorCommandMapper;
import org.griffins1884.griffinsim.sensors.SensorEmissionConfig;
import org.junit.jupiter.api.Test;

class MultiRobotScheduledScenarioRunnerTest {
  @Test
  void scheduledScenarioProducesDeterministicReplayAndMotion() {
    MultiRobotScenarioRunner runner = new MultiRobotScenarioRunner();
    MultiRobotRunResult first = runner.run(createScenario(), 4);
    MultiRobotRunResult second = runner.run(createScenario(), 4);

    assertArrayEquals(first.replayBytes(), second.replayBytes());
    assertTrue(first.finalWorldState().bodies().get(0).x() != 0.0);
  }

  private static MultiRobotScenario createScenario() {
    ControlHostConfig config = new ControlHostConfig(ClockMode.REAL_TIME, 5_000_000L, 20_000_000L, 16);
    return new MultiRobotScenario(
        "scheduled-sequence",
        config,
        new ImmutableWorldState(
            FrameHeader.current(0L, 0),
            List.of(new RigidBodyState("robot-a", 0.0, 0.0, 0.0, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0))),
        List.of(
            ScenarioEndpointFactory.scheduledPwmEndpoint(
                config,
                new ScheduledPwmRobotSpec(
                    "robot-a",
                    0,
                    new PwmCommandTimeline(
                        List.of(
                            new ScheduledPwmCommand(0, 0, 0.2),
                            new ScheduledPwmCommand(1, 2, 0.8),
                            new ScheduledPwmCommand(3, 3, -0.4)),
                        0.0),
                    1884L,
                    SensorEmissionConfig.immediate("robot-a", 0),
                    new SingleBodyActuatorCommandMapper("robot-a", 50.0, 10.0, 100.0, 0.0)))),
        FieldContactPresets.rebuilt2026Arena());
  }
}
