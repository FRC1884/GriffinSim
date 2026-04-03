package org.griffins1884.griffinsim.runtime;

import java.io.Closeable;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.UncheckedIOException;
import org.griffins1884.griffinsim.contracts.ActuatorFrame;
import org.griffins1884.griffinsim.contracts.ContactTelemetryFrame;
import org.griffins1884.griffinsim.contracts.FrameCodec;
import org.griffins1884.griffinsim.contracts.SensorFrame;
import org.griffins1884.griffinsim.contracts.WorldSnapshot;

public final class ReplayLogWriter implements Closeable {
  private static final int MAGIC = 0x4753494D;
  private final DataOutputStream out;

  public ReplayLogWriter(OutputStream outputStream) {
    this.out = new DataOutputStream(outputStream);
  }

  public void appendActuatorFrame(ActuatorFrame frame) {
    writeRecord(ReplayRecordType.ACTUATOR, FrameCodec.encodeActuatorFrame(frame));
  }

  public void appendSensorFrame(SensorFrame frame) {
    writeRecord(ReplayRecordType.SENSOR, FrameCodec.encodeSensorFrame(frame));
  }

  public void appendWorldSnapshot(WorldSnapshot snapshot) {
    writeRecord(ReplayRecordType.WORLD, FrameCodec.encodeWorldSnapshot(snapshot));
  }

  public void appendContactTelemetryFrame(ContactTelemetryFrame frame) {
    writeRecord(ReplayRecordType.CONTACT_TELEMETRY, FrameCodec.encodeContactTelemetryFrame(frame));
  }

  private void writeRecord(ReplayRecordType type, byte[] payload) {
    try {
      out.writeInt(MAGIC);
      out.writeInt(type.wireValue());
      out.writeInt(payload.length);
      out.write(payload);
      out.flush();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  @Override
  public void close() throws IOException {
    out.close();
  }
}
