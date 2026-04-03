package org.griffins1884.griffinsim.frc.nativebridge;

import java.util.Objects;
import org.griffins1884.griffinsim.contracts.ActuatorFrame;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.SensorFrame;
import org.griffins1884.griffinsim.frc.ActuatorFrameSupplier;
import org.griffins1884.griffinsim.frc.HalSimSensorFrameApplicator;
import org.griffins1884.griffinsim.frc.HalSimValueSink;
import org.griffins1884.griffinsim.frc.SensorFrameApplicator;
import org.griffins1884.griffinsim.contracts.PwmOutput;

public final class NativeBackedQueuedHalSimBridge implements ActuatorFrameSupplier, SensorFrameApplicator, AutoCloseable {
  private final NativeExtensionBindings bindings;
  private final HalSimSensorFrameApplicator sensorApplicator;

  public NativeBackedQueuedHalSimBridge(
      NativeExtensionBindings bindings,
      NativeExtensionRuntimeConfig runtimeConfig,
      HalSimValueSink valueSink) {
    this.bindings = Objects.requireNonNull(bindings);
    this.sensorApplicator = new HalSimSensorFrameApplicator(Objects.requireNonNull(valueSink));
    bindings.initExtension();
    bindings.setConfig(runtimeConfig);
  }

  @Override
  public void apply(SensorFrame frame) {
    sensorApplicator.apply(frame);
  }

  @Override
  public ActuatorFrame capture(FrameHeader header) {
    java.util.Map<Integer, NativePwmEvent> latestByChannel = new java.util.LinkedHashMap<>();
    while (true) {
      var next = bindings.dequeuePwmEvent();
      if (next.isEmpty()) {
        break;
      }
      NativePwmEvent event = next.orElseThrow();
      latestByChannel.put(event.channel(), event);
    }
    return new ActuatorFrame(
        header,
        latestByChannel.values().stream()
            .sorted(java.util.Comparator.comparingInt(NativePwmEvent::channel))
            .map(event -> new PwmOutput(event.channel(), event.value()))
            .toList());
  }

  public boolean enqueuePwmEvent(int channel, double value, long sequenceNumber) {
    return bindings.enqueuePwmEvent(channel, value, sequenceNumber);
  }

  public int registerPwmSpeedCallbacks(int channelCount) {
    return bindings.registerPwmSpeedCallbacks(channelCount);
  }

  public void unregisterPwmSpeedCallbacks() {
    bindings.unregisterPwmSpeedCallbacks();
  }

  public int registeredPwmCallbackCount() {
    return bindings.registeredPwmCallbackCount();
  }

  public int queueSize() {
    return bindings.queueSize();
  }

  @Override
  public void close() {
    bindings.unregisterPwmSpeedCallbacks();
    bindings.close();
  }
}
