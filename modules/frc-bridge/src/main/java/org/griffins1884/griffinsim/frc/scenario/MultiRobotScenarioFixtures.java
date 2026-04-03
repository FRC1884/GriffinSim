package org.griffins1884.griffinsim.frc.scenario;

import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.griffins1884.griffinsim.frc.ClockMode;
import org.griffins1884.griffinsim.frc.ControlHostConfig;
import org.griffins1884.griffinsim.physics.AugmentedActuatorCommandMapper;
import org.griffins1884.griffinsim.physics.BodyCollisionProfile;
import org.griffins1884.griffinsim.physics.BodyMotionProfile;
import org.griffins1884.griffinsim.physics.CompositeContactGenerator;
import org.griffins1884.griffinsim.physics.FieldContactPresets;
import org.griffins1884.griffinsim.physics.ImmutableWorldState;
import org.griffins1884.griffinsim.physics.MaterialProfiles;
import org.griffins1884.griffinsim.physics.PairwiseBodyContactGenerator;
import org.griffins1884.griffinsim.physics.SingleBodyActuatorCommandMapper;
import org.griffins1884.griffinsim.sensors.SensorEmissionConfig;

public final class MultiRobotScenarioFixtures {
  private MultiRobotScenarioFixtures() {}

  public static MultiRobotScenario twoRobotHeadOn() {
    ControlHostConfig config = new ControlHostConfig(ClockMode.REAL_TIME, 5_000_000L, 20_000_000L, 16);
    return new MultiRobotScenario(
        "two-robot-head-on",
        config,
        new ImmutableWorldState(
            FrameHeader.current(0L, 0),
            List.of(
                new RigidBodyState("robot-a", 0.0, 0.0, 0.0, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0),
                new RigidBodyState("robot-b", 0.6, 0.0, 0.0, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0))),
        List.of(
            ScenarioEndpointFactory.fixedPwmEndpoint(
                config,
                new FixedPwmRobotSpec(
                    "robot-a", 0, 0.5, 1884L, SensorEmissionConfig.immediate("robot-a", 0),
                    new SingleBodyActuatorCommandMapper("robot-a", 50.0, 10.0, 100.0, 0.0))),
            ScenarioEndpointFactory.fixedPwmEndpoint(
                config,
                new FixedPwmRobotSpec(
                    "robot-b", 1, -0.5, 1885L, SensorEmissionConfig.immediate("robot-b", 1),
                    new SingleBodyActuatorCommandMapper("robot-b", 50.0, 10.0, 100.0, 0.0)))),
        new CompositeContactGenerator(
            List.of(
                FieldContactPresets.rebuilt2026Arena(),
                new PairwiseBodyContactGenerator(
                    List.of(
                        new BodyCollisionProfile("robot-a", 0.4, 0.4, 0.4, MaterialProfiles.DEFAULT_BODY),
                        new BodyCollisionProfile("robot-b", 0.4, 0.4, 0.4, MaterialProfiles.DEFAULT_BODY))))));
  }

  public static MultiRobotScenario robotPushesGamepiece() {
    ControlHostConfig config = new ControlHostConfig(ClockMode.REAL_TIME, 5_000_000L, 20_000_000L, 16);
    return new MultiRobotScenario(
        "robot-pushes-gamepiece",
        config,
        new ImmutableWorldState(
            FrameHeader.current(0L, 0),
            List.of(
                new RigidBodyState("robot", 0.0, 0.0, 0.0, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0),
                new RigidBodyState("gamepiece", 0.3, 0.0, 0.0, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0))),
        List.of(
            ScenarioEndpointFactory.fixedPwmEndpoint(
                config,
                new FixedPwmRobotSpec(
                    "robot",
                    0,
                    0.5,
                    1884L,
                    SensorEmissionConfig.immediate("robot", 0),
                    new AugmentedActuatorCommandMapper(
                        new SingleBodyActuatorCommandMapper("robot", 50.0, 10.0, 100.0, 0.0),
                        List.of(new BodyMotionProfile("gamepiece", 3.0, 1.0)))))),
        new CompositeContactGenerator(
            List.of(
                FieldContactPresets.rebuilt2026Arena(),
                new PairwiseBodyContactGenerator(
                    List.of(
                        new BodyCollisionProfile("robot", 0.4, 0.4, 0.4, MaterialProfiles.DEFAULT_BODY),
                        new BodyCollisionProfile("gamepiece", 0.2, 0.2, 0.2, MaterialProfiles.HDPE))))));
  }
}
