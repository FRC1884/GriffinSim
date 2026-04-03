package org.griffins1884.griffinsim.frc.nativebridge;

import java.nio.file.Files;
import java.nio.file.Path;

public record NativeExtensionLoadPlan(Path libraryPath, NativeExtensionRuntimeConfig runtimeConfig) {
  public NativeExtensionLoadPlan {
    if (libraryPath == null) {
      throw new NullPointerException("libraryPath");
    }
    if (runtimeConfig == null) {
      throw new NullPointerException("runtimeConfig");
    }
  }

  public boolean artifactExists() {
    return Files.exists(libraryPath);
  }

  public static NativeExtensionLoadPlan forRepo(Path repoRoot, NativeExtensionRuntimeConfig runtimeConfig) {
    return new NativeExtensionLoadPlan(NativeExtensionArtifactLocator.firstExisting(repoRoot), runtimeConfig);
  }
}
