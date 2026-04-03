package org.griffins1884.griffinsim.frc.nativebridge;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.simulation.PWMDataJNI;
import java.io.IOException;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Duration;
import java.util.Locale;
import java.util.LinkedHashSet;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

class PanamaWpilibNativeExtensionRuntimeTest {
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
  void loadsExtensionViaHalsimEnvironmentAndCapturesLivePwmDataCallbackFlow() throws Exception {
    Path library = ensureWpilibNativeLibrary();

    ProcessBuilder processBuilder =
        new ProcessBuilder(
            javaExecutable().toString(),
            "--add-modules",
            "jdk.incubator.foreign",
            "--enable-native-access=ALL-UNNAMED",
            "-Djava.library.path=" + wpilibNativeLibraryDirectory(),
            "-cp",
            childClasspath(),
            HarnessMain.class.getName(),
            library.toAbsolutePath().toString());
    processBuilder.directory(repoRoot().toFile());
    processBuilder.environment().put("HALSIM_EXTENSIONS", library.toAbsolutePath().toString());
    appendNativeLibraryPath(processBuilder.environment(), wpilibNativeLibraryDirectory().toString());
    processBuilder.redirectErrorStream(true);

    Process process = processBuilder.start();
    String output = readFully(process.getInputStream());
    int exitCode = process.waitFor();

    assertEquals(0, exitCode, output);
    assertTrue(output.contains("autoloaded=true"), output);
    assertTrue(output.contains("registered=1"), output);
    assertTrue(output.contains("event=0:0.42"), output);
  }

  private static Path javaExecutable() {
    String executableName = System.getProperty("os.name", "").toLowerCase().contains("win") ? "java.exe" : "java";
    return Path.of(System.getProperty("java.home")).resolve("bin").resolve(executableName);
  }

  private static String readFully(InputStream inputStream) throws Exception {
    try (inputStream; ByteArrayOutputStream output = new ByteArrayOutputStream()) {
      inputStream.transferTo(output);
      return output.toString(StandardCharsets.UTF_8);
    }
  }

  private static String childClasspath() throws Exception {
    LinkedHashSet<String> entries = new LinkedHashSet<>();
    String currentClasspath =
        System.getProperty("griffinsim.testRuntimeClasspath", System.getProperty("java.class.path", ""));
    if (!currentClasspath.isBlank()) {
      for (String entry : currentClasspath.split(java.util.regex.Pattern.quote(File.pathSeparator))) {
        if (!entry.isBlank()) {
          entries.add(entry);
        }
      }
    }
    return String.join(File.pathSeparator, entries);
  }

  private static Path wpilibNativeLibraryDirectory() throws IOException {
    Path nativeRoot = repoRoot().resolve("build").resolve("wpilib-hal-sdk").resolve("2026.2.1").resolve("native");
    try (var stream = Files.walk(nativeRoot)) {
      return stream
          .filter(Files::isRegularFile)
          .filter(
              path ->
                  path.getFileName().toString().equals("libwpiHaljni.dylib")
                      || path.getFileName().toString().equals("libwpiHaljni.so")
                      || path.getFileName().toString().equals("wpiHaljni.dll"))
          .map(Path::getParent)
          .findFirst()
          .orElseThrow(() -> new IOException("Could not locate wpiHaljni in " + nativeRoot));
    }
  }

  private static void appendNativeLibraryPath(java.util.Map<String, String> environment, String path) {
    String variableName =
        System.getProperty("os.name", "").toLowerCase().contains("mac")
            ? "DYLD_LIBRARY_PATH"
            : "LD_LIBRARY_PATH";
    String current = environment.get(variableName);
    if (current == null || current.isBlank()) {
      environment.put(variableName, path);
      return;
    }
    environment.put(variableName, path + File.pathSeparator + current);
  }

  public static final class HarnessMain {
    private HarnessMain() {}

    public static void main(String[] args) throws Exception {
      if (args.length != 1) {
        throw new IllegalArgumentException("expected native library path");
      }

      Path library = Path.of(args[0]);
      if (!HAL.initialize(500, 0)) {
        throw new IllegalStateException("HAL.initialize failed");
      }

      try (PanamaNativeExtensionBindings bindings = PanamaNativeExtensionBindings.load(library)) {
        boolean autoloaded = bindings.enqueuePwmEvent(7, 0.25, 1L);
        System.out.println("autoloaded=" + autoloaded);
        if (!autoloaded) {
          throw new IllegalStateException("extension was not initialized through HALSIM_EXTENSIONS");
        }

        bindings.resetPwmQueue();
        bindings.setConfig(new NativeExtensionRuntimeConfig(16, true, 0.02, 0.005));
        int registered = bindings.registerPwmSpeedCallbacks(1);
        System.out.println("registered=" + registered);
        if (registered != 1 || bindings.registeredPwmCallbackCount() != 1) {
          throw new IllegalStateException("expected one registered PWM callback");
        }

        AtomicReference<Double> javaCallbackValue = new AtomicReference<>();
        int javaCallbackUid =
            PWMDataJNI.registerSpeedCallback(
                0,
                (name, value) -> javaCallbackValue.set(value.getDouble()),
                true);
        try {
          PWMDataJNI.setSpeed(0, 0.42);
          Thread.sleep(50L);
          System.out.println(
              "simSpeed=" + String.format(Locale.ROOT, "%.2f", PWMDataJNI.getSpeed(0)));
          Double javaValue = javaCallbackValue.get();
          if (javaValue != null) {
            System.out.println("javaCallback=" + String.format(Locale.ROOT, "%.2f", javaValue));
          }
          NativePwmEvent event = waitForEvent(bindings, 0, 0.42);
          System.out.println(
              "event="
                  + event.channel()
                  + ":"
                  + String.format(Locale.ROOT, "%.2f", event.value()));
        } finally {
          PWMDataJNI.cancelSpeedCallback(0, javaCallbackUid);
          bindings.unregisterPwmSpeedCallbacks();
        }
      } finally {
        HAL.shutdown();
      }
    }

    private static NativePwmEvent waitForEvent(
        PanamaNativeExtensionBindings bindings, int expectedChannel, double expectedValue)
        throws Exception {
      long deadline = System.nanoTime() + Duration.ofSeconds(1).toNanos();
      while (System.nanoTime() < deadline) {
        var next = bindings.dequeuePwmEvent();
        if (next.isPresent()) {
          NativePwmEvent event = next.orElseThrow();
          if (event.channel() == expectedChannel && Math.abs(event.value() - expectedValue) < 1e-9) {
            return event;
          }
        }
        Thread.sleep(10L);
      }
      throw new IllegalStateException("timed out waiting for PWM callback event");
    }
  }
}
