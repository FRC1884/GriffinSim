package org.griffins1884.griffinsim.frc.scenario;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.griffins1884.griffinsim.frc.ClockMode;
import org.griffins1884.griffinsim.frc.ControlHostConfig;
import org.griffins1884.griffinsim.physics.FieldContactPresets;
import org.griffins1884.griffinsim.physics.ImmutableWorldState;
import org.griffins1884.griffinsim.physics.TriChannelActuatorCommandMapper;
import org.griffins1884.griffinsim.sensors.SensorEmissionConfig;
import org.junit.jupiter.api.Test;

class MultiRobotVelocityPoseControllerFixtureTest {
  @Test
  void velocityAwarePoseFixtureProducesDeterministicReplay() {
    MultiRobotScenarioRunner runner = new MultiRobotScenarioRunner();
    MultiRobotRunResult first = runner.run(createScenario(), 4);
    MultiRobotRunResult second = runner.run(createScenario(), 4);

    assertArrayEquals(first.replayBytes(), second.replayBytes());
    RigidBodyState body = first.finalWorldState().bodies().get(0);
    assertTrue(body.x() != 0.0 || body.y() != 0.0 || body.wz() != 0.0);
    assertEquals(4, first.contactTelemetryFrames().size());
  }

  private static MultiRobotScenario createScenario() {
    ControlHostConfig config = new ControlHostConfig(ClockMode.REAL_TIME, 5_000_000L, 20_000_000L, 16);
    return new MultiRobotScenario(
        "velocity-pose-controller-fixture",
        config,
        new ImmutableWorldState(
            FrameHeader.current(0L, 0),
            List.of(new RigidBodyState("robot-a", 0.0, 0.0, 0.0, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0))),
        List.of(
            ScenarioEndpointFactory.scheduledPoseEndpoint(
                config,
                new ScheduledPoseRobotSpec(
                    "robot-a",
                    0,
                    1,
                    2,
                    HolonomicVelocityPoseFollowerCompiler.compile(
                        new HolonomicVelocityPoseFollowerSpec(
                            List.of(
                                new PoseSetpoint2d(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2),
                                new PoseSetpoint2d(1, 1.0, 0.5, 1.0, 0.3, 0.2, 0.1, 0.8),
                                new PoseSetpoint2d(3, -0.5, -0.25, -0.5, -0.2, -0.1, -0.05, 0.4)),
                            0.0,
                            0.0,
                            0.0,
                            1.2,
                            0.8,
                            0.5,
                            0.3,
                            0.9,
                            0.2,
                            0.1,
                            0.01,
                            0.01)),
                    1884L,
                    SensorEmissionConfig.immediate("robot-a", 0),
                    new TriChannelActuatorCommandMapper("robot-a", 0, 1, 2, 50.0, 10.0, 100.0, 20.0)))) ,
        FieldContactPresets.rebuilt2026Arena());
  }
}
