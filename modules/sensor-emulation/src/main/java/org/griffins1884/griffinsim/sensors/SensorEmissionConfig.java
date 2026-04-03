package org.griffins1884.griffinsim.sensors;

public record SensorEmissionConfig(
    String bodyId,
    int encoderChannel,
    long latencyNanos,
    double positionScale,
    double velocityScale,
    double encoderNoiseStdDev,
    double yawNoiseStdDev) {
  public SensorEmissionConfig {
    if (bodyId == null || bodyId.isBlank()) {
      throw new IllegalArgumentException("bodyId must be present");
    }
    if (encoderChannel < 0) {
      throw new IllegalArgumentException("encoderChannel must be non-negative");
    }
    if (latencyNanos < 0) {
      throw new IllegalArgumentException("latencyNanos must be non-negative");
    }
  }

  public static SensorEmissionConfig immediate(String bodyId, int encoderChannel) {
    return new SensorEmissionConfig(bodyId, encoderChannel, 0L, 1.0, 1.0, 0.0, 0.0);
  }
}
