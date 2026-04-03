package org.griffins1884.griffinsim.frc.nativebridge;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class PanamaWpilibNativeExtensionBindingsTest {
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

  private static Path ensureWpilibNativeLibrary() throws Exception {
    Path root = repoRoot();
    Path library = NativeExtensionArtifactLocator.defaultWpilibBuildOutput(root);
    if (Files.exists(library)) {
      return library;
    }
    Process build = new ProcessBuilder("bash", "scripts/build_native_extension_with_wpilib.sh")
        .directory(root.toFile())
        .inheritIO()
        .start();
    if (build.waitFor() != 0) {
      throw new IOException("WPILib-linked native extension build failed");
    }
    if (!Files.exists(library)) {
      throw new IOException("WPILib-linked native extension artifact not found after build");
    }
    return library;
  }

  @Test
  void loadsWpilibLinkedNativeLibraryWhenPresent() throws Exception {
    Path library = ensureWpilibNativeLibrary();

    try (PanamaNativeExtensionBindings bindings = PanamaNativeExtensionBindings.load(library)) {
      assertEquals("GriffinSimHalSimExtension", bindings.extensionName());
      assertEquals(0, bindings.initExtension());
      assertTrue(bindings.setConfig(new NativeExtensionRuntimeConfig(16, true, 0.02, 0.005)));
      assertEquals(16, bindings.getConfig().queueCapacity());
      assertTrue(bindings.enqueuePwmEvent(2, -0.3, 7L));
      NativePwmEvent event = bindings.dequeuePwmEvent().orElseThrow();
      assertEquals(2, event.channel());
      assertEquals(-0.3, event.value(), 1e-9);
      assertEquals(7L, event.sequenceNumber());
    }
  }
}
