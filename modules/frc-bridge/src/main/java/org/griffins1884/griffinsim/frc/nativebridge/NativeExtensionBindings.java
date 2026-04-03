package org.griffins1884.griffinsim.frc.nativebridge;

import java.util.Optional;

public interface NativeExtensionBindings extends AutoCloseable {
  int initExtension();

  void shutdownExtension();

  String extensionName();

  boolean setConfig(NativeExtensionRuntimeConfig config);

  NativeExtensionRuntimeConfig getConfig();

  boolean enqueuePwmEvent(int channel, double value, long sequenceNumber);

  Optional<NativePwmEvent> dequeuePwmEvent();

  void resetPwmQueue();

  int queueCapacity();

  int queueSize();

  int registerPwmSpeedCallbacks(int channelCount);

  void unregisterPwmSpeedCallbacks();

  int registeredPwmCallbackCount();

  @Override
  void close();
}
