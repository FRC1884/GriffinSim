package org.griffins1884.griffinsim.frc.nativebridge;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.ImuInput;
import org.griffins1884.griffinsim.contracts.SensorFrame;
import org.griffins1884.griffinsim.frc.HalSimValueSink;
import org.junit.jupiter.api.Test;

class NativeExtensionIntegrationSupportTest {
  @Test
  void createsNativeBackedBridgeWithCallbackRegistrationLifecycle() {
    RecordingBindings bindings = new RecordingBindings();
    RecordingSink sink = new RecordingSink();
    NativeExtensionRuntimeConfig runtimeConfig = NativeExtensionRuntimeConfig.defaultLockstep();

    try (NativeBackedQueuedHalSimBridge bridge =
        NativeExtensionIntegrationSupport.createBridgeWithPwmCallbacks(
            bindings, runtimeConfig, sink, 4)) {
      assertEquals(1, bindings.initCalls);
      assertEquals(runtimeConfig, bindings.config);
      assertEquals(4, bindings.lastRegisteredChannelCount);
      assertEquals(4, bridge.registeredPwmCallbackCount());

      assertTrue(bridge.enqueuePwmEvent(0, 0.5, 1));
      assertTrue(bridge.enqueuePwmEvent(1, -0.2, 2));
      var frame = bridge.capture(FrameHeader.current(20_000_000L, 1));
      assertEquals(List.of(0, 1), frame.pwmOutputs().stream().map(output -> output.channel()).toList());
      assertEquals(List.of(0.5, -0.2), frame.pwmOutputs().stream().map(output -> output.value()).toList());

      bridge.apply(
          new SensorFrame(
              FrameHeader.current(20_000_000L, 1),
              List.of(new org.griffins1884.griffinsim.contracts.EncoderInput(0, 1.0, 2.0)),
              new ImuInput(1, 2, 3, 4, 5, 6, 7, 8, 9)));
      assertEquals(List.of("enc:0:1.0:2.0", "imu:1.0:2.0:3.0"), sink.events);
    }

    assertEquals(1, bindings.unregisterCalls);
    assertTrue(bindings.closed);
  }

  private static final class RecordingBindings implements NativeExtensionBindings {
    private final ArrayDeque<NativePwmEvent> queue = new ArrayDeque<>();
    private int initCalls;
    private NativeExtensionRuntimeConfig config;
    private int lastRegisteredChannelCount;
    private int unregisterCalls;
    private boolean closed;

    @Override
    public int initExtension() {
      initCalls++;
      return 0;
    }

    @Override
    public void shutdownExtension() {}

    @Override
    public String extensionName() {
      return "recording";
    }

    @Override
    public boolean setConfig(NativeExtensionRuntimeConfig config) {
      this.config = config;
      return true;
    }

    @Override
    public NativeExtensionRuntimeConfig getConfig() {
      return config;
    }

    @Override
    public boolean enqueuePwmEvent(int channel, double value, long sequenceNumber) {
      return queue.offer(new NativePwmEvent(channel, value, sequenceNumber));
    }

    @Override
    public Optional<NativePwmEvent> dequeuePwmEvent() {
      return Optional.ofNullable(queue.poll());
    }

    @Override
    public void resetPwmQueue() {
      queue.clear();
    }

    @Override
    public int queueCapacity() {
      return 32;
    }

    @Override
    public int queueSize() {
      return queue.size();
    }

    @Override
    public int registerPwmSpeedCallbacks(int channelCount) {
      lastRegisteredChannelCount = channelCount;
      return channelCount;
    }

    @Override
    public void unregisterPwmSpeedCallbacks() {
      unregisterCalls++;
      lastRegisteredChannelCount = 0;
    }

    @Override
    public int registeredPwmCallbackCount() {
      return lastRegisteredChannelCount;
    }

    @Override
    public void close() {
      closed = true;
      shutdownExtension();
    }
  }

  private static final class RecordingSink implements HalSimValueSink {
    private final List<String> events = new ArrayList<>();

    @Override
    public void setEncoder(int channel, double position, double velocity) {
      events.add("enc:" + channel + ":" + position + ":" + velocity);
    }

    @Override
    public void setImu(ImuInput imuInput) {
      events.add("imu:" + imuInput.yaw() + ":" + imuInput.pitch() + ":" + imuInput.roll());
    }
  }
}
