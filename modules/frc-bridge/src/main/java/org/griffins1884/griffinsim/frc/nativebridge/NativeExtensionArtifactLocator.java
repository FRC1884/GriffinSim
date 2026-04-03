package org.griffins1884.griffinsim.frc.nativebridge;

import java.nio.file.Files;
import java.nio.file.Path;

public final class NativeExtensionArtifactLocator {
  private NativeExtensionArtifactLocator() {}

  public static String libraryFileNameForCurrentPlatform() {
    String os = System.getProperty("os.name", "").toLowerCase();
    if (os.contains("mac")) {
      return "libgriffinsim-halsim-extension.dylib";
    }
    if (os.contains("win")) {
      return "griffinsim-halsim-extension.dll";
    }
    return "libgriffinsim-halsim-extension.so";
  }

  public static Path defaultBuildOutput(Path repoRoot) {
    return repoRoot.resolve("build").resolve("native-halsim-extension").resolve(libraryFileNameForCurrentPlatform());
  }

  public static Path defaultWpilibBuildOutput(Path repoRoot) {
    return repoRoot.resolve("build").resolve("native-halsim-extension-wpilib").resolve(libraryFileNameForCurrentPlatform());
  }

  public static Path firstExisting(Path repoRoot) {
    Path wpilib = defaultWpilibBuildOutput(repoRoot);
    if (Files.exists(wpilib)) {
      return wpilib;
    }
    return defaultBuildOutput(repoRoot);
  }
}
