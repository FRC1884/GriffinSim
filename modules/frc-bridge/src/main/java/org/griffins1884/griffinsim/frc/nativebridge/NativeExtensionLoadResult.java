package org.griffins1884.griffinsim.frc.nativebridge;

import java.nio.file.Path;

public record NativeExtensionLoadResult(
    boolean success,
    Path libraryPath,
    String message,
    NativeExtensionRuntimeConfig runtimeConfig) {}
