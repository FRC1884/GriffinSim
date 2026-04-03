package org.griffins1884.griffinsim.contracts;

public record ImuInput(
    double yaw,
    double pitch,
    double roll,
    double yawRate,
    double pitchRate,
    double rollRate,
    double accelX,
    double accelY,
    double accelZ) {
  public static ImuInput zero() {
    return new ImuInput(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }
}
