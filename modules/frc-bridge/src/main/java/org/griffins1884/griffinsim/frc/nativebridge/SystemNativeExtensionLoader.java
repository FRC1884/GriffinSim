package org.griffins1884.griffinsim.frc.nativebridge;

import java.nio.file.Path;
import java.util.Objects;

public final class SystemNativeExtensionLoader implements NativeExtensionLoader {
  @Override
  public void load(Path libraryPath) {
    Objects.requireNonNull(libraryPath, "libraryPath");
    System.load(libraryPath.toAbsolutePath().toString());
  }
}
