package org.griffins1884.griffinsim.frc.nativebridge;

import java.nio.file.Path;

public final class NativeExtensionReleasePaths {
  private NativeExtensionReleasePaths() {}

  public static Path releaseArtifactsNativeDir(Path repoRoot) {
    return repoRoot.resolve("build").resolve("release-artifacts").resolve("native-bin");
  }

  public static Path publishStagingNativeDir(Path repoRoot) {
    return repoRoot.resolve("build").resolve("publish-staging").resolve("native-bin");
  }
}
