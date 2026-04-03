package org.griffins1884.griffinsim.frc;

import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicLong;
import org.griffins1884.griffinsim.contracts.ActuatorFrame;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.PwmOutput;
import org.griffins1884.griffinsim.contracts.SensorFrame;
import org.griffins1884.griffinsim.runtime.DeterministicRingBuffer;

public final class QueuedHalSimBridge implements ActuatorFrameSupplier, SensorFrameApplicator {
  private final DeterministicRingBuffer<HalSimPwmEvent> pwmEvents;
  private final HalSimSensorFrameApplicator sensorApplicator;
  private final AtomicLong sequenceNumbers = new AtomicLong();

  public QueuedHalSimBridge(int queueCapacity, HalSimValueSink valueSink) {
    this.pwmEvents = new DeterministicRingBuffer<>(queueCapacity);
    this.sensorApplicator = new HalSimSensorFrameApplicator(Objects.requireNonNull(valueSink));
  }

  public boolean onPwmChanged(int channel, double value) {
    return pwmEvents.offer(new HalSimPwmEvent(channel, value, sequenceNumbers.getAndIncrement()));
  }

  @Override
  public ActuatorFrame capture(FrameHeader header) {
    Map<Integer, HalSimPwmEvent> latestByChannel = new LinkedHashMap<>();
    pwmEvents.snapshot().stream()
        .sorted(Comparator.comparingLong(HalSimPwmEvent::sequenceNumber))
        .forEach(event -> latestByChannel.put(event.channel(), event));
    pwmEvents.drainTo(ignored -> {});
    return new ActuatorFrame(
        header,
        latestByChannel.values().stream()
            .sorted(Comparator.comparingInt(HalSimPwmEvent::channel))
            .map(event -> new PwmOutput(event.channel(), event.value()))
            .toList());
  }

  @Override
  public void apply(SensorFrame frame) {
    sensorApplicator.apply(frame);
  }
}
