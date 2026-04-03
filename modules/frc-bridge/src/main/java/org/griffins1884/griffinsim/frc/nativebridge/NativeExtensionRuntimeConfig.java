package org.griffins1884.griffinsim.frc.nativebridge;

public record NativeExtensionRuntimeConfig(
    int queueCapacity,
    boolean lockstepEnabled,
    double controlStepSeconds,
    double physicsStepSeconds) {
  public NativeExtensionRuntimeConfig {
    if (queueCapacity <= 0) {
      throw new IllegalArgumentException("queueCapacity must be positive");
    }
    if (controlStepSeconds <= 0.0 || physicsStepSeconds <= 0.0) {
      throw new IllegalArgumentException("step seconds must be positive");
    }
  }

  public static NativeExtensionRuntimeConfig defaultLockstep() {
    return new NativeExtensionRuntimeConfig(256, true, 0.020, 0.005);
  }
}
