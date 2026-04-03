package org.griffins1884.griffinsim.frc.nativebridge;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class PanamaNativeExtensionBindingsTest {
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

  private static Path ensureNativeLibrary() throws Exception {
    Path root = repoRoot();
    Path library = NativeExtensionArtifactLocator.defaultBuildOutput(root);
    if (Files.exists(library)) {
      return library;
    }
    Process configure = new ProcessBuilder("cmake", "-S", "native/halsim-extension", "-B", "build/native-halsim-extension")
        .directory(root.toFile())
        .inheritIO()
        .start();
    if (configure.waitFor() != 0) {
      throw new IOException("cmake configure failed");
    }
    Process build = new ProcessBuilder("cmake", "--build", "build/native-halsim-extension")
        .directory(root.toFile())
        .inheritIO()
        .start();
    if (build.waitFor() != 0) {
      throw new IOException("cmake build failed");
    }
    return library;
  }

  @Test
  void bindsAndExercisesStandaloneNativeAbi() throws Exception {
    Path library = ensureNativeLibrary();

    try (PanamaNativeExtensionBindings bindings = PanamaNativeExtensionBindings.load(library)) {
      assertEquals("GriffinSimHalSimExtension", bindings.extensionName());
      assertEquals(0, bindings.initExtension());
      assertTrue(bindings.setConfig(new NativeExtensionRuntimeConfig(8, true, 0.02, 0.005)));
      assertEquals(8, bindings.getConfig().queueCapacity());
      assertTrue(bindings.enqueuePwmEvent(1, 0.75, 42L));
      assertEquals(1, bindings.queueSize());
      NativePwmEvent event = bindings.dequeuePwmEvent().orElseThrow();
      assertEquals(1, event.channel());
      assertEquals(0.75, event.value(), 1e-9);
      assertEquals(42L, event.sequenceNumber());
      assertFalse(bindings.dequeuePwmEvent().isPresent());
      bindings.resetPwmQueue();
      assertEquals(0, bindings.queueSize());
      assertEquals(0, bindings.registerPwmSpeedCallbacks(4));
      assertEquals(0, bindings.registeredPwmCallbackCount());
    }
  }
}
