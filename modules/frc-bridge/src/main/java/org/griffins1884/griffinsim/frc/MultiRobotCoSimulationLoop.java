package org.griffins1884.griffinsim.frc;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import org.griffins1884.griffinsim.contracts.ActuatorFrame;
import org.griffins1884.griffinsim.contracts.ContactTelemetry;
import org.griffins1884.griffinsim.contracts.ContactTelemetryFrame;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.SensorFrame;
import org.griffins1884.griffinsim.contracts.WorldSnapshot;
import org.griffins1884.griffinsim.physics.BodyCommand;
import org.griffins1884.griffinsim.physics.ContactEvent;
import org.griffins1884.griffinsim.physics.ContactGenerator;
import org.griffins1884.griffinsim.physics.DeterministicPhysicsWorld;
import org.griffins1884.griffinsim.physics.ImmutableWorldState;
import org.griffins1884.griffinsim.physics.PhysicsStepRequest;
import org.griffins1884.griffinsim.physics.WorldSnapshots;
import org.griffins1884.griffinsim.runtime.ReplayLogWriter;

public final class MultiRobotCoSimulationLoop {
  private final ControlHostConfig config;
  private final DeterministicPhysicsWorld physicsWorld;
  private final List<MultiRobotEndpoint> endpoints;
  private final ContactGenerator contactGenerator;
  private final ReplayLogWriter replayLogWriter;
  private ImmutableWorldState currentWorldState;
  private List<ContactEvent> lastContactEvents;
  private ContactTelemetryFrame lastContactTelemetryFrame;

  public MultiRobotCoSimulationLoop(
      ControlHostConfig config,
      ImmutableWorldState initialWorldState,
      DeterministicPhysicsWorld physicsWorld,
      List<MultiRobotEndpoint> endpoints,
      ContactGenerator contactGenerator,
      ReplayLogWriter replayLogWriter) {
    this.config = Objects.requireNonNull(config);
    this.currentWorldState = Objects.requireNonNull(initialWorldState);
    this.physicsWorld = Objects.requireNonNull(physicsWorld);
    this.endpoints =
        List.copyOf((endpoints == null ? List.<MultiRobotEndpoint>of() : endpoints).stream()
            .sorted(Comparator.comparing(MultiRobotEndpoint::bodyId))
            .toList());
    this.contactGenerator = Objects.requireNonNull(contactGenerator);
    this.replayLogWriter = replayLogWriter;
    this.lastContactEvents = List.of();
    this.lastContactTelemetryFrame = new ContactTelemetryFrame(initialWorldState.header(), List.of());
  }

  public ImmutableWorldState currentWorldState() {
    return currentWorldState;
  }

  public List<ContactEvent> lastContactEvents() {
    return lastContactEvents;
  }

  public ContactTelemetryFrame lastContactTelemetryFrame() {
    return lastContactTelemetryFrame;
  }

  public ImmutableWorldState advanceOneTick() {
    WorldSnapshot worldSnapshot = WorldSnapshots.fromWorldState(currentWorldState);
    for (MultiRobotEndpoint endpoint : endpoints) {
      endpoint.sensorEmulator().observe(worldSnapshot);
      List<SensorFrame> sensorFrames = endpoint.sensorEmulator().releaseReady(worldSnapshot.header().simTimeNanos());
      for (SensorFrame sensorFrame : sensorFrames) {
        endpoint.controlHost().enqueueSensorFrame(sensorFrame);
        if (replayLogWriter != null) {
          replayLogWriter.appendSensorFrame(sensorFrame);
        }
      }
    }

    List<BodyCommand> combinedCommands = new ArrayList<>();
    List<ActuatorFrame> actuatorFrames = new ArrayList<>();
    for (MultiRobotEndpoint endpoint : endpoints) {
      ActuatorFrame actuatorFrame = endpoint.controlHost().runOneTick();
      actuatorFrames.add(actuatorFrame);
      combinedCommands.addAll(endpoint.actuatorCommandMapper().map(actuatorFrame, currentWorldState));
      if (replayLogWriter != null) {
        replayLogWriter.appendActuatorFrame(actuatorFrame);
      }
    }

    if (actuatorFrames.isEmpty()) {
      throw new IllegalStateException("MultiRobotCoSimulationLoop requires at least one endpoint");
    }

    int substeps = (int) (config.controlStepNanos() / config.physicsStepNanos());
    long simTimeNanos = actuatorFrames.get(0).header().simTimeNanos() - config.controlStepNanos();
    for (int substep = 1; substep <= substeps; substep++) {
      simTimeNanos += config.physicsStepNanos();
      FrameHeader stepHeader =
          new FrameHeader(
              actuatorFrames.get(0).header().protocolVersion(),
              simTimeNanos,
              (actuatorFrames.get(0).header().stepId() * 1000) + substep);
      var stepResult =
          physicsWorld.stepDetailed(
              currentWorldState,
              PhysicsStepRequest.earthLike(
                  stepHeader,
                  config.physicsStepNanos(),
                  combinedCommands,
                  contactGenerator.generateContacts(currentWorldState)));
      currentWorldState = stepResult.worldState();
      lastContactEvents = stepResult.contactEvents();
      lastContactTelemetryFrame =
          new ContactTelemetryFrame(
              stepHeader,
              lastContactEvents.stream()
                  .map(event ->
                      new ContactTelemetry(
                          event.bodyId(),
                          event.contactId(),
                          event.penetrationMeters(),
                          event.normalSpeedBefore(),
                          event.tangentialSpeedBefore(),
                          event.tangentialSpeedAfter(),
                          event.slipRatio(),
                          event.frictionCoefficient(),
                          event.restitutionCoefficient(),
                          event.rollingFrictionCoefficient(),
                          event.torsionalFrictionCoefficient()))
                  .toList());
      if (replayLogWriter != null) {
        replayLogWriter.appendWorldSnapshot(WorldSnapshots.fromWorldState(currentWorldState));
        replayLogWriter.appendContactTelemetryFrame(lastContactTelemetryFrame);
      }
    }
    return currentWorldState;
  }
}
