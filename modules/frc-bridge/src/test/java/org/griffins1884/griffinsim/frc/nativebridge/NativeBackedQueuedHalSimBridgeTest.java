package org.griffins1884.griffinsim.frc.nativebridge;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.ImuInput;
import org.griffins1884.griffinsim.contracts.SensorFrame;
import org.griffins1884.griffinsim.frc.HalSimValueSink;
import org.junit.jupiter.api.Test;

class NativeBackedQueuedHalSimBridgeTest {
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
  void nativeBackedBridgeCapturesQueuedActuatorsAndAppliesSensors() throws Exception {
    Path library = NativeExtensionArtifactLocator.defaultBuildOutput(repoRoot());
    if (!Files.exists(library)) {
      Process configure = new ProcessBuilder("cmake", "-S", "native/halsim-extension", "-B", "build/native-halsim-extension")
          .directory(repoRoot().toFile())
          .inheritIO()
          .start();
      if (configure.waitFor() != 0) throw new IllegalStateException("cmake configure failed");
      Process build = new ProcessBuilder("cmake", "--build", "build/native-halsim-extension")
          .directory(repoRoot().toFile())
          .inheritIO()
          .start();
      if (build.waitFor() != 0) throw new IllegalStateException("cmake build failed");
    }

    RecordingSink sink = new RecordingSink();
    try (NativeBackedQueuedHalSimBridge bridge =
        NativeExtensionIntegrationSupport.createBridge(
            repoRoot(), NativeExtensionRuntimeConfig.defaultLockstep(), sink)) {
      assertTrue(bridge.enqueuePwmEvent(0, 0.5, 1));
      assertTrue(bridge.enqueuePwmEvent(1, -0.2, 2));
      assertEquals(2, bridge.queueSize());

      var frame = bridge.capture(FrameHeader.current(20_000_000L, 1));
      assertEquals(List.of(0, 1), frame.pwmOutputs().stream().map(output -> output.channel()).toList());
      assertEquals(List.of(0.5, -0.2), frame.pwmOutputs().stream().map(output -> output.value()).toList());
      assertEquals(0, bridge.queueSize());

      bridge.apply(new SensorFrame(FrameHeader.current(20_000_000L, 1), List.of(new org.griffins1884.griffinsim.contracts.EncoderInput(0, 1.0, 2.0)), new ImuInput(1, 2, 3, 4, 5, 6, 7, 8, 9)));
      assertEquals(List.of("enc:0:1.0:2.0", "imu:1.0:2.0:3.0"), sink.events);
    }
  }

  private static final class RecordingSink implements HalSimValueSink {
    private final List<String> events = new ArrayList<>();

    @Override
    public void setEncoder(int channel, double position, double velocity) {
      events.add("enc:" + channel + ":" + position + ":" + velocity);
    }

    @Override
    public void setImu(ImuInput imuInput) {
      events.add("imu:" + imuInput.yaw() + ":" + imuInput.pitch() + ":" + imuInput.roll());
    }
  }
}
