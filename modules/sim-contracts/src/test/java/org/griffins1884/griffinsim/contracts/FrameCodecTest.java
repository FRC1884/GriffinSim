package org.griffins1884.griffinsim.contracts;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import org.junit.jupiter.api.Test;

class FrameCodecTest {
  @Test
  void actuatorEncodingIsByteStableAndRoundTrips() {
    ActuatorFrame frame =
        new ActuatorFrame(
            FrameHeader.current(20_000_000L, 1),
            List.of(new PwmOutput(0, 0.5), new PwmOutput(1, -0.25)));

    byte[] first = FrameCodec.encodeActuatorFrame(frame);
    byte[] second = FrameCodec.encodeActuatorFrame(frame);

    assertArrayEquals(first, second);
    assertEquals(frame, FrameCodec.decodeActuatorFrame(first));
  }

  @Test
  void sensorAndWorldSnapshotRoundTrip() {
    SensorFrame sensorFrame =
        new SensorFrame(
            FrameHeader.current(25_000_000L, 2),
            List.of(new EncoderInput(0, 1.2, 3.4)),
            new ImuInput(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 9.8, 0.0, -9.8));
    WorldSnapshot snapshot =
        new WorldSnapshot(
            FrameHeader.current(25_000_000L, 2),
            List.of(new RigidBodyState("robot", 1, 2, 3, 1, 0, 0, 0, 4, 5, 6, 7, 8, 9)));

    assertEquals(sensorFrame, FrameCodec.decodeSensorFrame(FrameCodec.encodeSensorFrame(sensorFrame)));
    assertEquals(snapshot, FrameCodec.decodeWorldSnapshot(FrameCodec.encodeWorldSnapshot(snapshot)));

    ContactTelemetryFrame telemetryFrame =
        new ContactTelemetryFrame(
            FrameHeader.current(25_000_000L, 2),
            List.of(new ContactTelemetry("robot", "floor", 0.05, -1.0, 2.0, 1.0, 0.5, 0.5, 0.25, 0.1, 0.02)));
    assertEquals(telemetryFrame, FrameCodec.decodeContactTelemetryFrame(FrameCodec.encodeContactTelemetryFrame(telemetryFrame)));
  }
}
