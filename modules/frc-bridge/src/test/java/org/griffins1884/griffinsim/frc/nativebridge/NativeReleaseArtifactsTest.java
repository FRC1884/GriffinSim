package org.griffins1884.griffinsim.frc.nativebridge;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class NativeReleaseArtifactsTest {
  private static Path repoRoot() {
    Path current = Path.of("").toAbsolutePath();
    while (current != null && !Files.exists(current.resolve("settings.gradle"))) {
      current = current.getParent();
    }
    if (current == null) {
      throw new IllegalStateException("Could not locate repository root");
    }
    return current;
  }

  @Test
  void releaseArtifactDirectoriesAreResolvable() {
    Path root = repoRoot();
    Path releaseDir = NativeExtensionReleasePaths.releaseArtifactsNativeDir(root);
    Path publishDir = NativeExtensionReleasePaths.publishStagingNativeDir(root);

    assertTrue(releaseDir.toString().contains("release-artifacts"));
    assertTrue(publishDir.toString().contains("publish-staging"));
  }
}
