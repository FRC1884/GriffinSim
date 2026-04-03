package org.griffins1884.griffinsim.frc.nativebridge;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class NativeExtensionRuntimeBridgeTest {
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
  void reportsMissingArtifactWithoutAttemptingLoad() {
    NativeExtensionRuntimeBridge bridge = new NativeExtensionRuntimeBridge(path -> {
      throw new AssertionError("loader should not be invoked for missing artifact");
    });
    Path missing = repoRoot().resolve("build").resolve("missing-native.dylib");
    NativeExtensionLoadResult result =
        bridge.loadFromPlan(new NativeExtensionLoadPlan(missing, NativeExtensionRuntimeConfig.defaultLockstep()));

    assertFalse(result.success());
    assertTrue(result.message().contains("not found"));
  }

  @Test
  void delegatesToLoaderWhenArtifactExists() throws Exception {
    Path fake = Files.createTempFile("griffinsim-native", ".bin");
    final Path[] loaded = new Path[1];
    NativeExtensionRuntimeBridge bridge = new NativeExtensionRuntimeBridge(path -> loaded[0] = path);

    NativeExtensionLoadResult result =
        bridge.loadFromPlan(new NativeExtensionLoadPlan(fake, NativeExtensionRuntimeConfig.defaultLockstep()));

    assertTrue(result.success());
    assertEquals(fake, loaded[0]);
    Files.deleteIfExists(fake);
  }
}
