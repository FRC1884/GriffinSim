package org.griffins1884.griffinsim.frc.nativebridge;

import java.nio.file.Path;

public interface NativeExtensionLoader {
  void load(Path libraryPath);
}
