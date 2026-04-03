package org.griffins1884.griffinsim.frc.nativebridge;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class NativeExtensionScaffoldTest {
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
  void nativeExtensionScaffoldFilesExist() {
    Path root = repoRoot();
    assertTrue(Files.exists(root.resolve("native/halsim-extension/CMakeLists.txt")));
    assertTrue(Files.exists(root.resolve("native/halsim-extension/include/griffinsim_halsim_extension.h")));
    assertTrue(Files.exists(root.resolve("native/halsim-extension/src/griffinsim_halsim_extension.cpp")));
    assertTrue(Files.exists(root.resolve("docs/architecture/native-halsim-extension.md")));
  }

  @Test
  void nativeHeaderAndSourceContainExpectedAbiSymbols() throws Exception {
    Path root = repoRoot();
    String header = Files.readString(root.resolve("native/halsim-extension/include/griffinsim_halsim_extension.h"));
    String source = Files.readString(root.resolve("native/halsim-extension/src/griffinsim_halsim_extension.cpp"));

    assertTrue(header.contains("GriffinSim_EnqueuePwmEvent"));
    assertTrue(header.contains("GriffinSim_DequeuePwmEvent"));
    assertTrue(header.contains("GriffinSim_SetExtensionConfig"));
    assertTrue(header.contains("GriffinSim_RegisterPwmSpeedCallbacks"));
    assertTrue(source.contains("HALSIM_InitExtension"));
    assertTrue(source.contains("GriffinSim_QueueCapacity"));
    assertTrue(source.contains("GriffinSim_RegisterPwmSpeedCallbacks"));
    assertTrue(source.contains("HALSIM_RegisterPWMSpeedCallback"));
    assertTrue(source.contains("std::array<GriffinSimPwmEvent"));
  }
}
