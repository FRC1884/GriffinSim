package org.griffins1884.griffinsim.frc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.ImuInput;
import org.griffins1884.griffinsim.contracts.SensorFrame;
import org.junit.jupiter.api.Test;

class QueuedHalSimBridgeTest {
  @Test
  void callbackThreadOnlyQueuesAndCaptureCollapsesLatestChannelValues() {
    RecordingSink sink = new RecordingSink();
    QueuedHalSimBridge bridge = new QueuedHalSimBridge(4, sink);

    assertTrue(bridge.onPwmChanged(2, 0.1));
    assertTrue(bridge.onPwmChanged(0, 0.5));
    assertTrue(bridge.onPwmChanged(2, 0.7));
    assertTrue(bridge.onPwmChanged(1, -0.2));
    assertFalse(bridge.onPwmChanged(3, 1.0));

    var frame = bridge.capture(FrameHeader.current(20_000_000L, 1));

    assertEquals(List.of(0, 1, 2), frame.pwmOutputs().stream().map(output -> output.channel()).toList());
    assertEquals(List.of(0.5, -0.2, 0.7), frame.pwmOutputs().stream().map(output -> output.value()).toList());
    assertTrue(bridge.capture(FrameHeader.current(40_000_000L, 2)).pwmOutputs().isEmpty());
  }

  @Test
  void sensorApplicatorRejectsStaleFrames() {
    RecordingSink sink = new RecordingSink();
    QueuedHalSimBridge bridge = new QueuedHalSimBridge(8, sink);

    bridge.apply(new SensorFrame(FrameHeader.current(20_000_000L, 1), List.of(new org.griffins1884.griffinsim.contracts.EncoderInput(0, 1.0, 2.0)), new ImuInput(1, 2, 3, 4, 5, 6, 7, 8, 9)));
    bridge.apply(new SensorFrame(FrameHeader.current(10_000_000L, 0), List.of(new org.griffins1884.griffinsim.contracts.EncoderInput(0, 9.0, 9.0)), new ImuInput(9, 9, 9, 9, 9, 9, 9, 9, 9)));

    assertEquals(List.of("enc:0:1.0:2.0", "imu:1.0:2.0:3.0"), sink.events);
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
