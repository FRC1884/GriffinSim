package org.griffins1884.griffinsim.frc;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import org.griffins1884.griffinsim.contracts.ActuatorFrame;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.SensorFrame;
import org.griffins1884.griffinsim.runtime.DeterministicRingBuffer;
import org.griffins1884.griffinsim.runtime.DeterministicScheduler;
import org.griffins1884.griffinsim.runtime.ScheduledTick;

public final class LockstepControlHost {
  private final ControlHostConfig config;
  private final SimTimeController timeController;
  private final SensorFrameApplicator sensorApplicator;
  private final RobotProgramLoop robotProgramLoop;
  private final ActuatorFrameSupplier actuatorFrameSupplier;
  private final DeterministicScheduler scheduler;
  private final DeterministicRingBuffer<SensorFrame> inboundSensorFrames;
  private final DeterministicRingBuffer<ActuatorFrame> outboundActuatorFrames;
  private int lastAppliedSensorStep;

  public LockstepControlHost(
      ControlHostConfig config,
      SimTimeController timeController,
      SensorFrameApplicator sensorApplicator,
      RobotProgramLoop robotProgramLoop,
      ActuatorFrameSupplier actuatorFrameSupplier) {
    this.config = Objects.requireNonNull(config);
    this.timeController = Objects.requireNonNull(timeController);
    this.sensorApplicator = Objects.requireNonNull(sensorApplicator);
    this.robotProgramLoop = Objects.requireNonNull(robotProgramLoop);
    this.actuatorFrameSupplier = Objects.requireNonNull(actuatorFrameSupplier);
    this.scheduler = new DeterministicScheduler(config.physicsStepNanos(), config.controlStepNanos());
    this.inboundSensorFrames = new DeterministicRingBuffer<>(config.queueCapacity());
    this.outboundActuatorFrames = new DeterministicRingBuffer<>(config.queueCapacity());
    this.lastAppliedSensorStep = -1;

    if (config.clockMode() == ClockMode.LOCKSTEP) {
      this.timeController.pause();
    } else {
      this.timeController.resume();
    }
  }

  public boolean enqueueSensorFrame(SensorFrame frame) {
    if (frame.header().stepId() <= lastAppliedSensorStep) {
      return false;
    }
    return inboundSensorFrames.offer(frame);
  }

  public ActuatorFrame runOneTick() {
    ScheduledTick tick = scheduler.nextTick();
    selectSensorFrameFor(tick.stepId()).ifPresent(frame -> {
      sensorApplicator.apply(frame);
      lastAppliedSensorStep = frame.header().stepId();
    });

    if (config.clockMode() == ClockMode.LOCKSTEP) {
      timeController.stepSeconds(config.controlStepNanos() / 1_000_000_000.0);
    } else {
      robotProgramLoop.runOneIteration();
    }

    ActuatorFrame actuatorFrame =
        actuatorFrameSupplier.capture(FrameHeader.current(tick.simTimeNanos(), tick.stepId()));
    if (!outboundActuatorFrames.offer(actuatorFrame)) {
      throw new IllegalStateException("Actuator queue overflow");
    }
    return actuatorFrame;
  }

  public Optional<ActuatorFrame> pollActuatorFrame() {
    return outboundActuatorFrames.poll();
  }

  private Optional<SensorFrame> selectSensorFrameFor(int controlStepId) {
    List<SensorFrame> futureFrames = new ArrayList<>();
    SensorFrame newestApplicable = null;
    Optional<SensorFrame> next;
    while ((next = inboundSensorFrames.poll()).isPresent()) {
      SensorFrame frame = next.orElseThrow();
      int stepId = frame.header().stepId();
      if (stepId <= lastAppliedSensorStep) {
        continue;
      }
      if (stepId <= controlStepId) {
        newestApplicable = frame;
      } else {
        futureFrames.add(frame);
      }
    }
    for (SensorFrame frame : futureFrames) {
      boolean offered = inboundSensorFrames.offer(frame);
      if (!offered) {
        throw new IllegalStateException("Sensor queue overflow while restoring future frames");
      }
    }
    return Optional.ofNullable(newestApplicable);
  }
}
