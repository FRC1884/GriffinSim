package org.griffins1884.griffinsim.frc.scenario;

import org.griffins1884.griffinsim.frc.ClockMode;
import org.griffins1884.griffinsim.frc.ControlHostConfig;
import org.griffins1884.griffinsim.frc.LockstepControlHost;
import org.griffins1884.griffinsim.frc.MultiRobotEndpoint;
import org.griffins1884.griffinsim.frc.NoOpHalSimValueSink;
import org.griffins1884.griffinsim.frc.NoOpSimTimeController;
import org.griffins1884.griffinsim.frc.QueuedHalSimBridge;
import org.griffins1884.griffinsim.frc.RobotProgramLoop;
import org.griffins1884.griffinsim.sensors.DeterministicSensorEmulator;

public final class ScenarioEndpointFactory {
  private ScenarioEndpointFactory() {}

  public static MultiRobotEndpoint fixedPwmEndpoint(ControlHostConfig config, FixedPwmRobotSpec spec) {
    return scheduledPwmEndpoint(
        config,
        new ScheduledPwmRobotSpec(
            spec.bodyId(),
            spec.pwmChannel(),
            PwmCommandTimeline.constant(spec.pwmValue()),
            spec.sensorSeed(),
            spec.sensorEmissionConfig(),
            spec.actuatorCommandMapper()));
  }

  public static MultiRobotEndpoint scheduledPwmEndpoint(
      ControlHostConfig config, ScheduledPwmRobotSpec spec) {
    QueuedHalSimBridge bridge = new QueuedHalSimBridge(config.queueCapacity(), new NoOpHalSimValueSink());
    RobotProgramLoop robotLoop = new TickDrivenRobotLoop(spec.pwmChannel(), spec.commandTimeline(), bridge);
    return endpoint(config, spec.bodyId(), spec.sensorSeed(), spec.sensorEmissionConfig(), spec.actuatorCommandMapper(), bridge, robotLoop);
  }

  public static MultiRobotEndpoint scheduledHolonomicEndpoint(
      ControlHostConfig config, ScheduledHolonomicRobotSpec spec) {
    QueuedHalSimBridge bridge = new QueuedHalSimBridge(config.queueCapacity(), new NoOpHalSimValueSink());
    RobotProgramLoop robotLoop =
        new HolonomicTickDrivenRobotLoop(spec.xPwmChannel(), spec.yPwmChannel(), spec.commandPlan(), bridge);
    return endpoint(config, spec.bodyId(), spec.sensorSeed(), spec.sensorEmissionConfig(), spec.actuatorCommandMapper(), bridge, robotLoop);
  }

  public static MultiRobotEndpoint scheduledPoseEndpoint(
      ControlHostConfig config, ScheduledPoseRobotSpec spec) {
    QueuedHalSimBridge bridge = new QueuedHalSimBridge(config.queueCapacity(), new NoOpHalSimValueSink());
    RobotProgramLoop robotLoop =
        new PoseTickDrivenRobotLoop(spec.xPwmChannel(), spec.yPwmChannel(), spec.thetaPwmChannel(), spec.commandPlan(), bridge);
    return endpoint(config, spec.bodyId(), spec.sensorSeed(), spec.sensorEmissionConfig(), spec.actuatorCommandMapper(), bridge, robotLoop);
  }

  private static MultiRobotEndpoint endpoint(
      ControlHostConfig config,
      String bodyId,
      long sensorSeed,
      org.griffins1884.griffinsim.sensors.SensorEmissionConfig sensorEmissionConfig,
      org.griffins1884.griffinsim.physics.ActuatorCommandMapper actuatorCommandMapper,
      QueuedHalSimBridge bridge,
      RobotProgramLoop robotLoop) {
    LockstepControlHost host =
        new LockstepControlHost(
            new ControlHostConfig(
                ClockMode.REAL_TIME,
                config.physicsStepNanos(),
                config.controlStepNanos(),
                config.queueCapacity()),
            new NoOpSimTimeController(),
            bridge,
            robotLoop,
            bridge);
    return new MultiRobotEndpoint(
        bodyId,
        host,
        new DeterministicSensorEmulator(sensorEmissionConfig, sensorSeed),
        actuatorCommandMapper);
  }

  private static final class TickDrivenRobotLoop implements RobotProgramLoop {
    private final int pwmChannel;
    private final PwmCommandTimeline timeline;
    private final QueuedHalSimBridge bridge;
    private int tick;

    private TickDrivenRobotLoop(int pwmChannel, PwmCommandTimeline timeline, QueuedHalSimBridge bridge) {
      this.pwmChannel = pwmChannel;
      this.timeline = timeline;
      this.bridge = bridge;
      this.tick = 0;
    }

    @Override
    public void runOneIteration() {
      bridge.onPwmChanged(pwmChannel, timeline.valueAtTick(tick));
      tick++;
    }
  }

  private static final class HolonomicTickDrivenRobotLoop implements RobotProgramLoop {
    private final int xPwmChannel;
    private final int yPwmChannel;
    private final HolonomicPwmCommandPlan commandPlan;
    private final QueuedHalSimBridge bridge;
    private int tick;

    private HolonomicTickDrivenRobotLoop(
        int xPwmChannel, int yPwmChannel, HolonomicPwmCommandPlan commandPlan, QueuedHalSimBridge bridge) {
      this.xPwmChannel = xPwmChannel;
      this.yPwmChannel = yPwmChannel;
      this.commandPlan = commandPlan;
      this.bridge = bridge;
      this.tick = 0;
    }

    @Override
    public void runOneIteration() {
      bridge.onPwmChanged(xPwmChannel, commandPlan.xTimeline().valueAtTick(tick));
      bridge.onPwmChanged(yPwmChannel, commandPlan.yTimeline().valueAtTick(tick));
      tick++;
    }
  }

  private static final class PoseTickDrivenRobotLoop implements RobotProgramLoop {
    private final int xPwmChannel;
    private final int yPwmChannel;
    private final int thetaPwmChannel;
    private final HolonomicPosePwmCommandPlan commandPlan;
    private final QueuedHalSimBridge bridge;
    private int tick;

    private PoseTickDrivenRobotLoop(
        int xPwmChannel,
        int yPwmChannel,
        int thetaPwmChannel,
        HolonomicPosePwmCommandPlan commandPlan,
        QueuedHalSimBridge bridge) {
      this.xPwmChannel = xPwmChannel;
      this.yPwmChannel = yPwmChannel;
      this.thetaPwmChannel = thetaPwmChannel;
      this.commandPlan = commandPlan;
      this.bridge = bridge;
      this.tick = 0;
    }

    @Override
    public void runOneIteration() {
      bridge.onPwmChanged(xPwmChannel, commandPlan.xTimeline().valueAtTick(tick));
      bridge.onPwmChanged(yPwmChannel, commandPlan.yTimeline().valueAtTick(tick));
      bridge.onPwmChanged(thetaPwmChannel, commandPlan.thetaTimeline().valueAtTick(tick));
      tick++;
    }
  }
}
