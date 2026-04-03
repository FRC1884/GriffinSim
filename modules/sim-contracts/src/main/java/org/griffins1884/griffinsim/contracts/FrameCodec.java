package org.griffins1884.griffinsim.contracts;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

public final class FrameCodec {
  private FrameCodec() {}

  public static byte[] encodeActuatorFrame(ActuatorFrame frame) {
    try {
      ByteArrayOutputStream byteStream = new ByteArrayOutputStream();
      DataOutputStream out = new DataOutputStream(byteStream);
      writeHeader(out, frame.header());
      out.writeInt(frame.pwmOutputs().size());
      for (PwmOutput pwmOutput : frame.pwmOutputs()) {
        out.writeInt(pwmOutput.channel());
        out.writeDouble(pwmOutput.value());
      }
      out.flush();
      return byteStream.toByteArray();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  public static ActuatorFrame decodeActuatorFrame(byte[] bytes) {
    try {
      DataInputStream in = new DataInputStream(new ByteArrayInputStream(bytes));
      FrameHeader header = readHeader(in);
      int count = in.readInt();
      List<PwmOutput> outputs = new ArrayList<>(count);
      for (int i = 0; i < count; i++) {
        outputs.add(new PwmOutput(in.readInt(), in.readDouble()));
      }
      return new ActuatorFrame(header, outputs);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  public static byte[] encodeSensorFrame(SensorFrame frame) {
    try {
      ByteArrayOutputStream byteStream = new ByteArrayOutputStream();
      DataOutputStream out = new DataOutputStream(byteStream);
      writeHeader(out, frame.header());
      out.writeInt(frame.encoders().size());
      for (EncoderInput encoder : frame.encoders()) {
        out.writeInt(encoder.channel());
        out.writeDouble(encoder.position());
        out.writeDouble(encoder.velocity());
      }
      writeImu(out, frame.imu());
      out.flush();
      return byteStream.toByteArray();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  public static SensorFrame decodeSensorFrame(byte[] bytes) {
    try {
      DataInputStream in = new DataInputStream(new ByteArrayInputStream(bytes));
      FrameHeader header = readHeader(in);
      int count = in.readInt();
      List<EncoderInput> encoders = new ArrayList<>(count);
      for (int i = 0; i < count; i++) {
        encoders.add(new EncoderInput(in.readInt(), in.readDouble(), in.readDouble()));
      }
      return new SensorFrame(header, encoders, readImu(in));
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  public static byte[] encodeWorldSnapshot(WorldSnapshot snapshot) {
    try {
      ByteArrayOutputStream byteStream = new ByteArrayOutputStream();
      DataOutputStream out = new DataOutputStream(byteStream);
      writeHeader(out, snapshot.header());
      out.writeInt(snapshot.bodies().size());
      for (RigidBodyState body : snapshot.bodies()) {
        writeString(out, body.bodyId());
        out.writeDouble(body.x());
        out.writeDouble(body.y());
        out.writeDouble(body.z());
        out.writeDouble(body.qw());
        out.writeDouble(body.qx());
        out.writeDouble(body.qy());
        out.writeDouble(body.qz());
        out.writeDouble(body.vx());
        out.writeDouble(body.vy());
        out.writeDouble(body.vz());
        out.writeDouble(body.wx());
        out.writeDouble(body.wy());
        out.writeDouble(body.wz());
      }
      out.flush();
      return byteStream.toByteArray();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  public static WorldSnapshot decodeWorldSnapshot(byte[] bytes) {
    try {
      DataInputStream in = new DataInputStream(new ByteArrayInputStream(bytes));
      FrameHeader header = readHeader(in);
      int count = in.readInt();
      List<RigidBodyState> bodies = new ArrayList<>(count);
      for (int i = 0; i < count; i++) {
        bodies.add(
            new RigidBodyState(
                readString(in),
                in.readDouble(),
                in.readDouble(),
                in.readDouble(),
                in.readDouble(),
                in.readDouble(),
                in.readDouble(),
                in.readDouble(),
                in.readDouble(),
                in.readDouble(),
                in.readDouble(),
                in.readDouble(),
                in.readDouble(),
                in.readDouble()));
      }
      return new WorldSnapshot(header, bodies);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }


  public static byte[] encodeContactTelemetryFrame(ContactTelemetryFrame frame) {
    try {
      ByteArrayOutputStream byteStream = new ByteArrayOutputStream();
      DataOutputStream out = new DataOutputStream(byteStream);
      writeHeader(out, frame.header());
      out.writeInt(frame.contacts().size());
      for (ContactTelemetry contact : frame.contacts()) {
        writeString(out, contact.bodyId());
        writeString(out, contact.contactId());
        out.writeDouble(contact.penetrationMeters());
        out.writeDouble(contact.normalSpeedBefore());
        out.writeDouble(contact.tangentialSpeedBefore());
        out.writeDouble(contact.tangentialSpeedAfter());
        out.writeDouble(contact.slipRatio());
        out.writeDouble(contact.frictionCoefficient());
        out.writeDouble(contact.restitutionCoefficient());
        out.writeDouble(contact.rollingFrictionCoefficient());
        out.writeDouble(contact.torsionalFrictionCoefficient());
      }
      out.flush();
      return byteStream.toByteArray();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  public static ContactTelemetryFrame decodeContactTelemetryFrame(byte[] bytes) {
    try {
      DataInputStream in = new DataInputStream(new ByteArrayInputStream(bytes));
      FrameHeader header = readHeader(in);
      int count = in.readInt();
      List<ContactTelemetry> contacts = new ArrayList<>(count);
      for (int i = 0; i < count; i++) {
        contacts.add(new ContactTelemetry(
            readString(in),
            readString(in),
            in.readDouble(),
            in.readDouble(),
            in.readDouble(),
            in.readDouble(),
            in.readDouble(),
            in.readDouble(),
            in.readDouble(),
            in.readDouble(),
            in.readDouble()));
      }
      return new ContactTelemetryFrame(header, contacts);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  private static void writeHeader(DataOutputStream out, FrameHeader header) throws IOException {
    out.writeInt(header.protocolVersion().wireValue());
    out.writeLong(header.simTimeNanos());
    out.writeInt(header.stepId());
  }

  private static FrameHeader readHeader(DataInputStream in) throws IOException {
    return new FrameHeader(ProtocolVersion.fromWireValue(in.readInt()), in.readLong(), in.readInt());
  }

  private static void writeImu(DataOutputStream out, ImuInput imu) throws IOException {
    out.writeDouble(imu.yaw());
    out.writeDouble(imu.pitch());
    out.writeDouble(imu.roll());
    out.writeDouble(imu.yawRate());
    out.writeDouble(imu.pitchRate());
    out.writeDouble(imu.rollRate());
    out.writeDouble(imu.accelX());
    out.writeDouble(imu.accelY());
    out.writeDouble(imu.accelZ());
  }

  private static ImuInput readImu(DataInputStream in) throws IOException {
    return new ImuInput(
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble());
  }

  private static void writeString(DataOutputStream out, String value) throws IOException {
    byte[] bytes = value.getBytes(StandardCharsets.UTF_8);
    out.writeInt(bytes.length);
    out.write(bytes);
  }

  private static String readString(DataInputStream in) throws IOException {
    int length = in.readInt();
    byte[] bytes = in.readNBytes(length);
    return new String(bytes, StandardCharsets.UTF_8);
  }
}
