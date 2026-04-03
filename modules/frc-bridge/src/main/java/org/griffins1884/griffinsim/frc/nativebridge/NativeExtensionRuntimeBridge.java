package org.griffins1884.griffinsim.frc.nativebridge;

import java.nio.file.Path;
import java.util.Objects;

public final class NativeExtensionRuntimeBridge {
  private final NativeExtensionLoader loader;

  public NativeExtensionRuntimeBridge(NativeExtensionLoader loader) {
    this.loader = Objects.requireNonNull(loader);
  }

  public NativeExtensionLoadResult loadFromPlan(NativeExtensionLoadPlan plan) {
    Objects.requireNonNull(plan, "plan");
    Path libraryPath = plan.libraryPath();
    if (!plan.artifactExists()) {
      return new NativeExtensionLoadResult(
          false,
          libraryPath,
          "native extension artifact not found: " + libraryPath,
          plan.runtimeConfig());
    }
    try {
      loader.load(libraryPath);
      return new NativeExtensionLoadResult(
          true,
          libraryPath,
          "native extension loaded",
          plan.runtimeConfig());
    } catch (RuntimeException | UnsatisfiedLinkError e) {
      return new NativeExtensionLoadResult(
          false,
          libraryPath,
          "native extension load failed: " + e.getMessage(),
          plan.runtimeConfig());
    }
  }
}
