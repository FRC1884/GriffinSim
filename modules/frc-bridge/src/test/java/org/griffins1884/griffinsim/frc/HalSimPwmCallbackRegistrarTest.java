package org.griffins1884.griffinsim.frc;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.simulation.NotifyCallback;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class HalSimPwmCallbackRegistrarTest {
  @Test
  void registersCallbacksAndCancelsThemOnClose() {
    RecordingSink sink = new RecordingSink();
    QueuedHalSimBridge bridge = new QueuedHalSimBridge(8, sink);
    FakePwmCallbackSource source = new FakePwmCallbackSource();

    try (HalSimPwmCallbackRegistrar registrar = new HalSimPwmCallbackRegistrar(source, bridge)) {
      registrar.registerChannels(3);
      source.callbacks.get(2).callback("speed", HALValue.makeDouble(0.75));
      source.callbacks.get(0).callback("speed", HALValue.makeDouble(-0.5));
      var frame = bridge.capture(org.griffins1884.griffinsim.contracts.FrameHeader.current(20_000_000L, 1));
      assertEquals(List.of(0, 2), frame.pwmOutputs().stream().map(output -> output.channel()).toList());
      assertEquals(List.of(-0.5, 0.75), frame.pwmOutputs().stream().map(output -> output.value()).toList());
    }

    assertEquals(
        List.of("register:0", "register:1", "register:2", "cancel:0:100", "cancel:1:101", "cancel:2:102"),
        source.events);
  }

  private static final class FakePwmCallbackSource implements PwmCallbackSource {
    private final List<String> events = new ArrayList<>();
    private final List<NotifyCallback> callbacks = new ArrayList<>();

    @Override
    public int registerSpeedCallback(int channel, NotifyCallback callback, boolean initialNotify) {
      callbacks.add(callback);
      events.add("register:" + channel);
      return 100 + channel;
    }

    @Override
    public void cancelSpeedCallback(int channel, int uid) {
      events.add("cancel:" + channel + ":" + uid);
    }
  }

  private static final class RecordingSink implements HalSimValueSink {
    @Override
    public void setEncoder(int channel, double position, double velocity) {}

    @Override
    public void setImu(org.griffins1884.griffinsim.contracts.ImuInput imuInput) {}
  }
}
