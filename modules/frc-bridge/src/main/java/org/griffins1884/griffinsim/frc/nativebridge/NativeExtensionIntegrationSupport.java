package org.griffins1884.griffinsim.frc.nativebridge;

import java.nio.file.Path;
import java.util.Objects;
import org.griffins1884.griffinsim.frc.HalSimValueSink;

public final class NativeExtensionIntegrationSupport {
  private NativeExtensionIntegrationSupport() {}

  public static NativeBackedQueuedHalSimBridge createBridge(
      NativeExtensionBindings bindings,
      NativeExtensionRuntimeConfig runtimeConfig,
      HalSimValueSink valueSink) {
    return new NativeBackedQueuedHalSimBridge(
        Objects.requireNonNull(bindings), runtimeConfig, valueSink);
  }

  public static NativeBackedQueuedHalSimBridge createBridge(
      Path repoRoot,
      NativeExtensionRuntimeConfig runtimeConfig,
      HalSimValueSink valueSink) {
    Objects.requireNonNull(repoRoot);
    NativeExtensionLoadPlan plan = NativeExtensionLoadPlan.forRepo(repoRoot, runtimeConfig);
    PanamaNativeExtensionBindings bindings = PanamaNativeExtensionBindings.load(plan.libraryPath());
    return createBridge(bindings, runtimeConfig, valueSink);
  }

  public static NativeBackedQueuedHalSimBridge createBridgeWithPwmCallbacks(
      NativeExtensionBindings bindings,
      NativeExtensionRuntimeConfig runtimeConfig,
      HalSimValueSink valueSink,
      int channelCount) {
    NativeBackedQueuedHalSimBridge bridge = createBridge(bindings, runtimeConfig, valueSink);
    bridge.registerPwmSpeedCallbacks(channelCount);
    return bridge;
  }

  public static NativeBackedQueuedHalSimBridge createBridgeWithPwmCallbacks(
      Path repoRoot,
      NativeExtensionRuntimeConfig runtimeConfig,
      HalSimValueSink valueSink,
      int channelCount) {
    NativeBackedQueuedHalSimBridge bridge = createBridge(repoRoot, runtimeConfig, valueSink);
    bridge.registerPwmSpeedCallbacks(channelCount);
    return bridge;
  }
}
