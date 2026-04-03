package org.griffins1884.griffinsim.runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.util.Arrays;
import java.util.List;
import org.griffins1884.griffinsim.contracts.ActuatorFrame;
import org.griffins1884.griffinsim.contracts.ContactTelemetry;
import org.griffins1884.griffinsim.contracts.ContactTelemetryFrame;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.PwmOutput;
import org.junit.jupiter.api.Test;

class ReplayLogTest {
  @Test
  void identicalFramesProduceIdenticalReplayBytes() throws Exception {
    ActuatorFrame frame =
        new ActuatorFrame(FrameHeader.current(20_000_000L, 1), List.of(new PwmOutput(0, 0.25)));

    byte[] first;
    byte[] second;
    try (ByteArrayOutputStream left = new ByteArrayOutputStream();
        ByteArrayOutputStream right = new ByteArrayOutputStream();
        ReplayLogWriter leftWriter = new ReplayLogWriter(left);
        ReplayLogWriter rightWriter = new ReplayLogWriter(right)) {
      leftWriter.appendActuatorFrame(frame);
      leftWriter.appendContactTelemetryFrame(new ContactTelemetryFrame(FrameHeader.current(20_000_000L, 1), List.of(new ContactTelemetry("robot", "floor", 0.05, -1.0, 2.0, 1.0, 0.5, 0.5, 0.25, 0.1, 0.02))));
      rightWriter.appendActuatorFrame(frame);
      rightWriter.appendContactTelemetryFrame(new ContactTelemetryFrame(FrameHeader.current(20_000_000L, 1), List.of(new ContactTelemetry("robot", "floor", 0.05, -1.0, 2.0, 1.0, 0.5, 0.5, 0.25, 0.1, 0.02))));
      first = left.toByteArray();
      second = right.toByteArray();
    }

    List<ReplayRecord> records = ReplayLogReader.readAll(new ByteArrayInputStream(first));
    assertEquals(2, records.size());
    assertEquals(ReplayRecordType.ACTUATOR, records.get(0).type());
    assertEquals(ReplayRecordType.CONTACT_TELEMETRY, records.get(1).type());
    assertTrue(Arrays.equals(first, second));
  }
}
