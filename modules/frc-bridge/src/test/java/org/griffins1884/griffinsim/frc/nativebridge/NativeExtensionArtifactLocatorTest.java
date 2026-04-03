package org.griffins1884.griffinsim.frc.nativebridge;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class NativeExtensionArtifactLocatorTest {
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
  void resolvesReasonableLibraryNamesAndPaths() {
    String fileName = NativeExtensionArtifactLocator.libraryFileNameForCurrentPlatform();
    assertTrue(fileName.contains("griffinsim-halsim-extension"));

    Path repoRoot = repoRoot();
    Path defaultBuild = NativeExtensionArtifactLocator.defaultBuildOutput(repoRoot);
    Path firstExisting = NativeExtensionArtifactLocator.firstExisting(repoRoot);

    assertTrue(defaultBuild.toString().contains("native-halsim-extension"));
    assertTrue(firstExisting.getFileName().toString().contains("griffinsim-halsim-extension"));
  }

  @Test
  void loadPlanTracksArtifactExistence() {
    NativeExtensionLoadPlan plan =
        NativeExtensionLoadPlan.forRepo(repoRoot(), NativeExtensionRuntimeConfig.defaultLockstep());
    assertTrue(plan.libraryPath().getFileName().toString().contains("griffinsim-halsim-extension"));
    assertTrue(plan.runtimeConfig().lockstepEnabled());
  }
}
