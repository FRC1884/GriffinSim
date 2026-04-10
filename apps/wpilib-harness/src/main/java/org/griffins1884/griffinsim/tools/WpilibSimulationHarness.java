package org.griffins1884.griffinsim.tools;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import org.griffins1884.griffinsim.contracts.ContactTelemetryFrame;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.griffins1884.griffinsim.frc.ClockMode;
import org.griffins1884.griffinsim.frc.ControlHostConfig;
import org.griffins1884.griffinsim.frc.DeterministicCoSimulationLoop;
import org.griffins1884.griffinsim.frc.HalSimPwmCallbackRegistrar;
import org.griffins1884.griffinsim.frc.HalSimValueSink;
import org.griffins1884.griffinsim.frc.JniPwmCallbackSource;
import org.griffins1884.griffinsim.frc.LockstepControlHost;
import org.griffins1884.griffinsim.frc.QueuedHalSimBridge;
import org.griffins1884.griffinsim.frc.SimTimeController;
import org.griffins1884.griffinsim.frc.WpiSimHooksTimeController;
import org.griffins1884.griffinsim.physics.ActuatorCommandMapper;
import org.griffins1884.griffinsim.physics.BodyCommand;
import org.griffins1884.griffinsim.physics.ContactGenerator;
import org.griffins1884.griffinsim.physics.DeterministicPhysicsWorld;
import org.griffins1884.griffinsim.physics.ImmutableWorldState;
import org.griffins1884.griffinsim.runtime.ReplayLogWriter;
import org.griffins1884.griffinsim.sensors.DeterministicSensorEmulator;

public final class WpilibSimulationHarness {
  private static final Duration ROBOT_START_TIMEOUT = Duration.ofSeconds(5);
  private static final Duration ROBOT_STOP_TIMEOUT = Duration.ofSeconds(5);

  private WpilibSimulationHarness() {}

  public static <T extends TimedRobot & BoundRobot> RunResult<T> run(
      Supplier<T> robotSupplier, HarnessSpec spec) {
    Objects.requireNonNull(robotSupplier, "robotSupplier");
    Objects.requireNonNull(spec, "spec");

    AtomicReference<T> robotRef = new AtomicReference<>();
    AtomicReference<Throwable> robotFailure = new AtomicReference<>();
    ByteArrayOutputStream replayStream = new ByteArrayOutputStream();
    ReplayLogWriter replayLogWriter = new ReplayLogWriter(replayStream);
    Thread robotThread =
        new Thread(
            () -> {
              try {
                RobotBase.startRobot(
                    () -> {
                      T robot = robotSupplier.get();
                      robotRef.set(robot);
                      return robot;
                    });
              } catch (Throwable failure) {
                robotFailure.compareAndSet(null, failure);
              }
            },
            "griffinsim-wpilib-robot");
    robotThread.setDaemon(true);

    T robot = null;
    HalSimPwmCallbackRegistrar registrar = null;
    boolean replayClosed = false;
    try {
      robotThread.start();
      robot = awaitRobot(robotRef, robotFailure);

      DriverStationSim.resetData();
      SimHooks.restartTiming();
      SimHooks.pauseTiming();

      QueuedHalSimBridge bridge =
          new QueuedHalSimBridge(spec.controlHostConfig().queueCapacity(), robot.halSimValueSink());
      registrar = new HalSimPwmCallbackRegistrar(new JniPwmCallbackSource(), bridge);
      registrar.registerChannels(spec.pwmChannelCount());

      LockstepControlHost controlHost =
          new LockstepControlHost(
              spec.controlHostConfig(),
              new DriverStationPacketTimeController(),
              bridge,
              () -> {},
              bridge);
      DeterministicCoSimulationLoop loop =
          new DeterministicCoSimulationLoop(
              spec.controlHostConfig(),
              spec.initialWorldState(),
              controlHost,
              spec.physicsWorld(),
              spec.sensorEmulator(),
              spec.actuatorCommandMapper(),
              spec.contactGenerator(),
              replayLogWriter);

      List<TraceStep> trace = new ArrayList<>();
      int executedTicks = 0;
      for (DriverStationPhase phase : spec.phases()) {
        applyPhase(phase);
        for (int tick = 0; tick < phase.ticks(); tick++) {
          assertRobotHealthy(robotFailure);
          loop.advanceOneTick();
          trace.add(
              TraceStep.capture(
                  phase.name(),
                  phase.mode(),
                  executedTicks,
                  spec.traceBodyId(),
                  loop.currentWorldState(),
                  loop.lastActuatorFrame(),
                  loop.lastAppliedCommands()));
          executedTicks++;
        }
      }
      assertRobotHealthy(robotFailure);
      replayLogWriter.close();
      replayClosed = true;
      return new RunResult<>(
          robot,
          loop.currentWorldState(),
          replayStream.toByteArray(),
          loop.lastContactTelemetryFrame(),
          trace,
          executedTicks);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IllegalStateException("Interrupted while running WPILib harness", e);
    } catch (IOException e) {
      throw new IllegalStateException("Failed to close replay output", e);
    } finally {
      safeClose(registrar);
      stopRobot(robot, robotThread);
      if (!replayClosed) {
        safeClose(replayLogWriter);
      }
      safeClose(robot);
      safeResetSimState();
      safeShutdownHal();
    }
  }

  private static <T extends TimedRobot & BoundRobot> T awaitRobot(
      AtomicReference<T> robotRef, AtomicReference<Throwable> robotFailure) throws InterruptedException {
    long deadlineNanos = System.nanoTime() + ROBOT_START_TIMEOUT.toNanos();
    while (System.nanoTime() < deadlineNanos) {
      assertRobotHealthy(robotFailure);
      T robot = robotRef.get();
      if (robot != null) {
        return robot;
      }
      Thread.sleep(10L);
    }
    throw new IllegalStateException("Timed out waiting for WPILib robot startup");
  }

  private static void assertRobotHealthy(AtomicReference<Throwable> robotFailure) {
    Throwable failure = robotFailure.get();
    if (failure != null) {
      throw new IllegalStateException("WPILib robot thread failed", failure);
    }
  }

  private static void applyPhase(DriverStationPhase phase) {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(phase.mode() == Mode.AUTONOMOUS);
    DriverStationSim.setTest(phase.mode() == Mode.TEST);
    DriverStationSim.setEnabled(phase.mode() != Mode.DISABLED);
    DriverStationSim.notifyNewData();
  }

  private static void stopRobot(RobotBase robot, Thread robotThread) {
    if (!robotThread.isAlive()) {
      return;
    }
    try {
      DriverStationSim.setEnabled(false);
      DriverStationSim.setAutonomous(false);
      DriverStationSim.setTest(false);
      DriverStationSim.setDsAttached(true);
      DriverStationSim.notifyNewData();
    } catch (Throwable ignored) {
      // Best effort shutdown path.
    }
    try {
      RobotBase.suppressExitWarning(true);
      HAL.exitMain();
    } catch (Throwable ignored) {
      // Best effort shutdown path.
    }
    join(robotThread, ROBOT_STOP_TIMEOUT);
    if (!robotThread.isAlive() && robot != null) {
      return;
    }
    if (robot != null) {
      try {
        robot.endCompetition();
      } catch (Throwable ignored) {
        // Best effort shutdown path.
      }
      join(robotThread, ROBOT_STOP_TIMEOUT);
    }
    if (robotThread.isAlive()) {
      throw new IllegalStateException("Timed out stopping WPILib robot thread");
    }
  }

  private static void join(Thread thread, Duration timeout) {
    try {
      thread.join(timeout.toMillis());
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IllegalStateException("Interrupted while waiting for WPILib robot shutdown", e);
    }
  }

  private static void safeResetSimState() {
    try {
      DriverStationSim.resetData();
      SimHooks.restartTiming();
      SimHooks.pauseTiming();
    } catch (Throwable ignored) {
      // Best effort cleanup.
    }
  }

  private static void safeShutdownHal() {
    try {
      HAL.shutdown();
    } catch (Throwable ignored) {
      // Best effort cleanup.
    }
  }

  private static void safeClose(AutoCloseable closeable) {
    if (closeable == null) {
      return;
    }
    try {
      closeable.close();
    } catch (Exception ignored) {
      // Best effort cleanup.
    }
  }

  public interface BoundRobot {
    HalSimValueSink halSimValueSink();
  }

  public enum Mode {
    DISABLED,
    TELEOP,
    AUTONOMOUS,
    TEST
  }

  public record DriverStationPhase(String name, Mode mode, int ticks) {
    public DriverStationPhase {
      if (name == null || name.isBlank()) {
        throw new IllegalArgumentException("name must be present");
      }
      Objects.requireNonNull(mode, "mode");
      if (ticks <= 0) {
        throw new IllegalArgumentException("ticks must be positive");
      }
    }

    public static DriverStationPhase disabled(String name, int ticks) {
      return new DriverStationPhase(name, Mode.DISABLED, ticks);
    }

    public static DriverStationPhase teleop(String name, int ticks) {
      return new DriverStationPhase(name, Mode.TELEOP, ticks);
    }

    public static DriverStationPhase autonomous(String name, int ticks) {
      return new DriverStationPhase(name, Mode.AUTONOMOUS, ticks);
    }

    public static DriverStationPhase test(String name, int ticks) {
      return new DriverStationPhase(name, Mode.TEST, ticks);
    }
  }

  public record HarnessSpec(
      ControlHostConfig controlHostConfig,
      ImmutableWorldState initialWorldState,
      DeterministicPhysicsWorld physicsWorld,
      DeterministicSensorEmulator sensorEmulator,
      ActuatorCommandMapper actuatorCommandMapper,
      ContactGenerator contactGenerator,
      String traceBodyId,
      int pwmChannelCount,
      List<DriverStationPhase> phases) {
    public HarnessSpec {
      Objects.requireNonNull(controlHostConfig, "controlHostConfig");
      Objects.requireNonNull(initialWorldState, "initialWorldState");
      physicsWorld = physicsWorld == null ? new DeterministicPhysicsWorld() : physicsWorld;
      Objects.requireNonNull(sensorEmulator, "sensorEmulator");
      Objects.requireNonNull(actuatorCommandMapper, "actuatorCommandMapper");
      Objects.requireNonNull(contactGenerator, "contactGenerator");
      if (traceBodyId == null || traceBodyId.isBlank()) {
        throw new IllegalArgumentException("traceBodyId must be present");
      }
      if (controlHostConfig.clockMode() != ClockMode.LOCKSTEP) {
        throw new IllegalArgumentException("WPILib harness requires lockstep clock mode");
      }
      if (pwmChannelCount <= 0) {
        throw new IllegalArgumentException("pwmChannelCount must be positive");
      }
      phases = List.copyOf(phases == null ? List.of() : phases);
      if (phases.isEmpty()) {
        throw new IllegalArgumentException("phases must be present");
      }
    }
  }

  public record RunResult<T extends TimedRobot & BoundRobot>(
      T robot,
      ImmutableWorldState finalWorldState,
      byte[] replayBytes,
      ContactTelemetryFrame lastContactTelemetryFrame,
      List<TraceStep> trace,
      int executedTicks) {
    public RunResult {
      Objects.requireNonNull(robot, "robot");
      Objects.requireNonNull(finalWorldState, "finalWorldState");
      replayBytes = replayBytes == null ? new byte[0] : replayBytes.clone();
      lastContactTelemetryFrame =
          lastContactTelemetryFrame == null
              ? new ContactTelemetryFrame(finalWorldState.header(), List.of())
              : lastContactTelemetryFrame;
      trace = List.copyOf(trace == null ? List.of() : trace);
      if (executedTicks <= 0) {
        throw new IllegalArgumentException("executedTicks must be positive");
      }
    }

    @Override
    public byte[] replayBytes() {
      return replayBytes.clone();
    }

    @Override
    public List<TraceStep> trace() {
      return trace;
    }
  }

  public record TraceStep(
      String phaseName,
      Mode mode,
      int controlTickIndex,
      long simTimeNanos,
      double x,
      double y,
      double headingRadians,
      double vx,
      double vy,
      double linearSpeedMetersPerSecond,
      double angularVelocityRadiansPerSecond,
      List<Double> pwmByChannel,
      List<BodyCommand> appliedCommands) {
    public TraceStep {
      if (phaseName == null || phaseName.isBlank()) {
        throw new IllegalArgumentException("phaseName must be present");
      }
      Objects.requireNonNull(mode, "mode");
      if (controlTickIndex < 0) {
        throw new IllegalArgumentException("controlTickIndex must be non-negative");
      }
      pwmByChannel = List.copyOf(pwmByChannel == null ? List.of() : pwmByChannel);
      appliedCommands = List.copyOf(appliedCommands == null ? List.of() : appliedCommands);
    }

    static TraceStep capture(
        String phaseName,
        Mode mode,
        int controlTickIndex,
        String traceBodyId,
        ImmutableWorldState worldState,
        org.griffins1884.griffinsim.contracts.ActuatorFrame actuatorFrame,
        List<BodyCommand> appliedCommands) {
      RigidBodyState body =
          worldState.bodies().stream()
              .filter(candidate -> candidate.bodyId().equals(traceBodyId))
              .findFirst()
              .orElseThrow(
                  () -> new IllegalArgumentException("Missing traced body in world state: " + traceBodyId));
      int maxChannel =
          actuatorFrame.pwmOutputs().stream()
              .mapToInt(output -> output.channel())
              .max()
              .orElse(-1);
      List<Double> pwmByChannel = new ArrayList<>();
      for (int channel = 0; channel <= maxChannel; channel++) {
        final int currentChannel = channel;
        double value =
            actuatorFrame.pwmOutputs().stream()
                .filter(output -> output.channel() == currentChannel)
                .mapToDouble(output -> output.value())
                .findFirst()
                .orElse(0.0);
        pwmByChannel.add(value);
      }
      return new TraceStep(
          phaseName,
          mode,
          controlTickIndex,
          worldState.header().simTimeNanos(),
          body.x(),
          body.y(),
          quaternionYaw(body),
          body.vx(),
          body.vy(),
          Math.hypot(body.vx(), body.vy()),
          body.wz(),
          pwmByChannel,
          appliedCommands);
    }
  }

  private static final class DriverStationPacketTimeController implements SimTimeController {
    private final WpiSimHooksTimeController delegate = new WpiSimHooksTimeController();

    @Override
    public void pause() {
      delegate.pause();
    }

    @Override
    public void resume() {
      delegate.resume();
    }

    @Override
    public void stepSeconds(double seconds) {
      DriverStationSim.notifyNewData();
      delegate.stepSeconds(seconds);
    }
  }

  private static double quaternionYaw(RigidBodyState body) {
    double sinyCosp = 2.0 * ((body.qw() * body.qz()) + (body.qx() * body.qy()));
    double cosyCosp = 1.0 - (2.0 * ((body.qy() * body.qy()) + (body.qz() * body.qz())));
    return Math.atan2(sinyCosp, cosyCosp);
  }
}
