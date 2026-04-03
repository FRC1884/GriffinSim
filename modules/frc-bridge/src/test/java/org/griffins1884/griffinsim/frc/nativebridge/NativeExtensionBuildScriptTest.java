package org.griffins1884.griffinsim.frc.nativebridge;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class NativeExtensionBuildScriptTest {
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
  void wpilibBuildScriptAndLoadingDocsExist() throws Exception {
    Path root = repoRoot();
    assertTrue(Files.exists(root.resolve("scripts/build_native_extension_with_wpilib.sh")));
    String cmake = Files.readString(root.resolve("native/halsim-extension/CMakeLists.txt"));
    assertTrue(cmake.contains("GRIFFINSIM_ENABLE_WPILIB_HALSIM"));
    assertTrue(cmake.contains("GRIFFINSIM_WPILIB_HAL_INCLUDE_DIR"));
    assertTrue(cmake.contains("GRIFFINSIM_WPILIB_HAL_LIBRARY"));
    assertTrue(Files.exists(root.resolve("docs/architecture/native-loading.md")));
  }
}
