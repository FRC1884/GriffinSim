package org.griffins1884.griffinsim.frc.nativebridge;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodType;
import java.nio.file.Path;
import java.util.Optional;
import jdk.incubator.foreign.CLinker;
import jdk.incubator.foreign.FunctionDescriptor;
import jdk.incubator.foreign.MemoryAccess;
import jdk.incubator.foreign.MemoryAddress;
import jdk.incubator.foreign.MemoryLayout;
import jdk.incubator.foreign.MemorySegment;
import jdk.incubator.foreign.ResourceScope;
import jdk.incubator.foreign.SymbolLookup;

public final class PanamaNativeExtensionBindings implements NativeExtensionBindings {
  private static final CLinker LINKER = CLinker.getInstance();
  private static final MemoryLayout PWM_EVENT_LAYOUT =
      MemoryLayout.structLayout(
          CLinker.C_INT.withName("channel"),
          MemoryLayout.paddingLayout(32),
          CLinker.C_DOUBLE.withName("value"),
          CLinker.C_LONG_LONG.withName("sequence_number"));
  private static final long PWM_EVENT_CHANNEL_OFFSET =
      PWM_EVENT_LAYOUT.byteOffset(MemoryLayout.PathElement.groupElement("channel"));
  private static final long PWM_EVENT_VALUE_OFFSET =
      PWM_EVENT_LAYOUT.byteOffset(MemoryLayout.PathElement.groupElement("value"));
  private static final long PWM_EVENT_SEQUENCE_OFFSET =
      PWM_EVENT_LAYOUT.byteOffset(MemoryLayout.PathElement.groupElement("sequence_number"));

  private static final MemoryLayout CONFIG_LAYOUT =
      MemoryLayout.structLayout(
          CLinker.C_INT.withName("queue_capacity"),
          CLinker.C_INT.withName("lockstep_enabled"),
          CLinker.C_DOUBLE.withName("control_step_seconds"),
          CLinker.C_DOUBLE.withName("physics_step_seconds"));
  private static final long CONFIG_QUEUE_OFFSET =
      CONFIG_LAYOUT.byteOffset(MemoryLayout.PathElement.groupElement("queue_capacity"));
  private static final long CONFIG_LOCKSTEP_OFFSET =
      CONFIG_LAYOUT.byteOffset(MemoryLayout.PathElement.groupElement("lockstep_enabled"));
  private static final long CONFIG_CONTROL_OFFSET =
      CONFIG_LAYOUT.byteOffset(MemoryLayout.PathElement.groupElement("control_step_seconds"));
  private static final long CONFIG_PHYSICS_OFFSET =
      CONFIG_LAYOUT.byteOffset(MemoryLayout.PathElement.groupElement("physics_step_seconds"));

  private final ResourceScope scope;
  private final MethodHandle initHandle;
  private final MethodHandle shutdownHandle;
  private final MethodHandle nameHandle;
  private final MethodHandle setConfigHandle;
  private final MethodHandle getConfigHandle;
  private final MethodHandle enqueueHandle;
  private final MethodHandle dequeueHandle;
  private final MethodHandle resetQueueHandle;
  private final MethodHandle queueCapacityHandle;
  private final MethodHandle queueSizeHandle;
  private final MethodHandle registerCallbacksHandle;
  private final MethodHandle unregisterCallbacksHandle;
  private final MethodHandle registeredCallbacksHandle;

  private PanamaNativeExtensionBindings(ResourceScope scope, SymbolLookup lookup) {
    this.scope = scope;
    this.initHandle = downcall(lookup, "HALSIM_InitExtension", MethodType.methodType(int.class), FunctionDescriptor.of(CLinker.C_INT));
    this.shutdownHandle = downcall(lookup, "HALSIM_ShutdownExtension", MethodType.methodType(void.class), FunctionDescriptor.ofVoid());
    this.nameHandle = downcall(lookup, "GriffinSim_HalSimExtensionName", MethodType.methodType(MemoryAddress.class), FunctionDescriptor.of(CLinker.C_POINTER));
    this.setConfigHandle = downcall(lookup, "GriffinSim_SetExtensionConfig", MethodType.methodType(int.class, MemoryAddress.class), FunctionDescriptor.of(CLinker.C_INT, CLinker.C_POINTER));
    this.getConfigHandle = downcall(lookup, "GriffinSim_GetExtensionConfig", MethodType.methodType(int.class, MemoryAddress.class), FunctionDescriptor.of(CLinker.C_INT, CLinker.C_POINTER));
    this.enqueueHandle = downcall(lookup, "GriffinSim_EnqueuePwmEvent", MethodType.methodType(int.class, int.class, double.class, long.class), FunctionDescriptor.of(CLinker.C_INT, CLinker.C_INT, CLinker.C_DOUBLE, CLinker.C_LONG_LONG));
    this.dequeueHandle = downcall(lookup, "GriffinSim_DequeuePwmEvent", MethodType.methodType(int.class, MemoryAddress.class), FunctionDescriptor.of(CLinker.C_INT, CLinker.C_POINTER));
    this.resetQueueHandle = downcall(lookup, "GriffinSim_ResetPwmQueue", MethodType.methodType(void.class), FunctionDescriptor.ofVoid());
    this.queueCapacityHandle = downcall(lookup, "GriffinSim_QueueCapacity", MethodType.methodType(int.class), FunctionDescriptor.of(CLinker.C_INT));
    this.queueSizeHandle = downcall(lookup, "GriffinSim_QueueSize", MethodType.methodType(int.class), FunctionDescriptor.of(CLinker.C_INT));
    this.registerCallbacksHandle = downcall(lookup, "GriffinSim_RegisterPwmSpeedCallbacks", MethodType.methodType(int.class, int.class), FunctionDescriptor.of(CLinker.C_INT, CLinker.C_INT));
    this.unregisterCallbacksHandle = downcall(lookup, "GriffinSim_UnregisterPwmSpeedCallbacks", MethodType.methodType(void.class), FunctionDescriptor.ofVoid());
    this.registeredCallbacksHandle = downcall(lookup, "GriffinSim_RegisteredPwmCallbackCount", MethodType.methodType(int.class), FunctionDescriptor.of(CLinker.C_INT));
  }

  public static PanamaNativeExtensionBindings load(Path libraryPath) {
    System.load(libraryPath.toAbsolutePath().toString());
    ResourceScope scope = ResourceScope.newSharedScope();
    return new PanamaNativeExtensionBindings(scope, SymbolLookup.loaderLookup());
  }

  @Override
  public int initExtension() {
    return invokeInt(initHandle);
  }

  @Override
  public void shutdownExtension() {
    invokeVoid(shutdownHandle);
  }

  @Override
  public String extensionName() {
    try {
      return CLinker.toJavaString((MemoryAddress) nameHandle.invokeExact());
    } catch (Throwable t) {
      throw new IllegalStateException("Failed to read extension name", t);
    }
  }

  @Override
  public boolean setConfig(NativeExtensionRuntimeConfig config) {
    try (ResourceScope local = ResourceScope.newConfinedScope()) {
      MemorySegment seg = MemorySegment.allocateNative(CONFIG_LAYOUT, local);
      MemoryAccess.setIntAtOffset(seg, CONFIG_QUEUE_OFFSET, config.queueCapacity());
      MemoryAccess.setIntAtOffset(seg, CONFIG_LOCKSTEP_OFFSET, config.lockstepEnabled() ? 1 : 0);
      MemoryAccess.setDoubleAtOffset(seg, CONFIG_CONTROL_OFFSET, config.controlStepSeconds());
      MemoryAccess.setDoubleAtOffset(seg, CONFIG_PHYSICS_OFFSET, config.physicsStepSeconds());
      return invokeInt(setConfigHandle, seg.address()) != 0;
    }
  }

  @Override
  public NativeExtensionRuntimeConfig getConfig() {
    try (ResourceScope local = ResourceScope.newConfinedScope()) {
      MemorySegment seg = MemorySegment.allocateNative(CONFIG_LAYOUT, local);
      if (invokeInt(getConfigHandle, seg.address()) == 0) {
        throw new IllegalStateException("Native config read failed");
      }
      return new NativeExtensionRuntimeConfig(
          MemoryAccess.getIntAtOffset(seg, CONFIG_QUEUE_OFFSET),
          MemoryAccess.getIntAtOffset(seg, CONFIG_LOCKSTEP_OFFSET) != 0,
          MemoryAccess.getDoubleAtOffset(seg, CONFIG_CONTROL_OFFSET),
          MemoryAccess.getDoubleAtOffset(seg, CONFIG_PHYSICS_OFFSET));
    }
  }

  @Override
  public boolean enqueuePwmEvent(int channel, double value, long sequenceNumber) {
    return invokeInt(enqueueHandle, channel, value, sequenceNumber) != 0;
  }

  @Override
  public Optional<NativePwmEvent> dequeuePwmEvent() {
    try (ResourceScope local = ResourceScope.newConfinedScope()) {
      MemorySegment seg = MemorySegment.allocateNative(PWM_EVENT_LAYOUT, local);
      if (invokeInt(dequeueHandle, seg.address()) == 0) {
        return Optional.empty();
      }
      return Optional.of(
          new NativePwmEvent(
              MemoryAccess.getIntAtOffset(seg, PWM_EVENT_CHANNEL_OFFSET),
              MemoryAccess.getDoubleAtOffset(seg, PWM_EVENT_VALUE_OFFSET),
              MemoryAccess.getLongAtOffset(seg, PWM_EVENT_SEQUENCE_OFFSET)));
    }
  }

  @Override
  public void resetPwmQueue() {
    invokeVoid(resetQueueHandle);
  }

  @Override
  public int queueCapacity() {
    return invokeInt(queueCapacityHandle);
  }

  @Override
  public int queueSize() {
    return invokeInt(queueSizeHandle);
  }

  @Override
  public int registerPwmSpeedCallbacks(int channelCount) {
    return invokeInt(registerCallbacksHandle, channelCount);
  }

  @Override
  public void unregisterPwmSpeedCallbacks() {
    invokeVoid(unregisterCallbacksHandle);
  }

  @Override
  public int registeredPwmCallbackCount() {
    return invokeInt(registeredCallbacksHandle);
  }

  @Override
  public void close() {
    shutdownExtension();
    scope.close();
  }

  private static MethodHandle downcall(
      SymbolLookup lookup, String symbol, MethodType methodType, FunctionDescriptor descriptor) {
    MemoryAddress address = lookup.lookup(symbol).orElseThrow(() -> new IllegalStateException("Missing native symbol: " + symbol));
    return LINKER.downcallHandle(address, methodType, descriptor);
  }

  private static int invokeInt(MethodHandle handle, Object... args) {
    try {
      return (int) handle.invokeWithArguments(args);
    } catch (Throwable t) {
      throw new IllegalStateException("Native invocation failed", t);
    }
  }

  private static void invokeVoid(MethodHandle handle, Object... args) {
    try {
      handle.invokeWithArguments(args);
    } catch (Throwable t) {
      throw new IllegalStateException("Native invocation failed", t);
    }
  }
}
