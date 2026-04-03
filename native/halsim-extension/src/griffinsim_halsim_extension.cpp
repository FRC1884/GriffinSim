#include "griffinsim_halsim_extension.h"

#include <array>
#include <mutex>

#ifdef GRIFFINSIM_ENABLE_WPILIB_HALSIM
#include "hal/simulation/PWMData.h"
#include "hal/simulation/NotifyListener.h"
#endif

namespace {
constexpr const char* kExtensionName = "GriffinSimHalSimExtension";
constexpr int kDefaultQueueCapacity = 256;
constexpr int kStorageCapacity = 1024;
constexpr int kMaxCallbackChannels = 64;

struct CallbackRegistration {
  bool active = false;
  int channel = -1;
  int uid = -1;
};

struct ExtensionState {
  GriffinSimExtensionConfig config{ kDefaultQueueCapacity, 1, 0.020, 0.005 };
  std::array<GriffinSimPwmEvent, kStorageCapacity> queue{};
  std::array<CallbackRegistration, kMaxCallbackChannels> callbacks{};
  uint64_t next_sequence_number = 0;
  int head = 0;
  int tail = 0;
  int size = 0;
  bool initialized = false;
  std::mutex mutex;
};

ExtensionState& State() {
  static ExtensionState state;
  return state;
}

int EffectiveCapacity(const ExtensionState& state) {
  if (state.config.queue_capacity <= 0) {
    return kDefaultQueueCapacity;
  }
  return state.config.queue_capacity > kStorageCapacity ? kStorageCapacity : state.config.queue_capacity;
}

void ResetQueueLocked(ExtensionState& state) {
  state.head = 0;
  state.tail = 0;
  state.size = 0;
}

int EnqueueLocked(ExtensionState& state, int channel, double value, uint64_t sequence_number) {
  const int capacity = EffectiveCapacity(state);
  if (state.size >= capacity) {
    return 0;
  }
  state.queue[state.tail] = GriffinSimPwmEvent{channel, value, sequence_number};
  state.tail = (state.tail + 1) % kStorageCapacity;
  state.size += 1;
  return 1;
}

#ifdef GRIFFINSIM_ENABLE_WPILIB_HALSIM
void GriffinSimPwmNotifyCallback(const char*, void* param, const struct HAL_Value* value) {
  auto* channel_ptr = static_cast<int*>(param);
  if (channel_ptr == nullptr || value == nullptr) {
    return;
  }
  auto& state = State();
  std::lock_guard<std::mutex> lock(state.mutex);
  if (!state.initialized) {
    return;
  }
  EnqueueLocked(state, *channel_ptr, value->data.v_double, state.next_sequence_number++);
}
#endif
}

extern "C" GRIFFINSIM_EXTENSION_API int HALSIM_InitExtension(void) {
  auto& state = State();
  std::lock_guard<std::mutex> lock(state.mutex);
  state.initialized = true;
  state.next_sequence_number = 0;
  ResetQueueLocked(state);
  return 0;
}

extern "C" GRIFFINSIM_EXTENSION_API void HALSIM_ShutdownExtension(void) {
  GriffinSim_UnregisterPwmSpeedCallbacks();
  auto& state = State();
  std::lock_guard<std::mutex> lock(state.mutex);
  state.initialized = false;
  state.next_sequence_number = 0;
  ResetQueueLocked(state);
}

extern "C" GRIFFINSIM_EXTENSION_API const char* GriffinSim_HalSimExtensionName(void) {
  return kExtensionName;
}

extern "C" GRIFFINSIM_EXTENSION_API int GriffinSim_SetExtensionConfig(const GriffinSimExtensionConfig* config) {
  if (config == nullptr) {
    return 0;
  }
  auto& state = State();
  std::lock_guard<std::mutex> lock(state.mutex);
  state.config = *config;
  if (state.size > EffectiveCapacity(state)) {
    ResetQueueLocked(state);
  }
  return 1;
}

extern "C" GRIFFINSIM_EXTENSION_API int GriffinSim_GetExtensionConfig(GriffinSimExtensionConfig* config_out) {
  if (config_out == nullptr) {
    return 0;
  }
  auto& state = State();
  std::lock_guard<std::mutex> lock(state.mutex);
  *config_out = state.config;
  return 1;
}

extern "C" GRIFFINSIM_EXTENSION_API int GriffinSim_EnqueuePwmEvent(int channel, double value, uint64_t sequence_number) {
  auto& state = State();
  std::lock_guard<std::mutex> lock(state.mutex);
  if (!state.initialized) {
    return 0;
  }
  return EnqueueLocked(state, channel, value, sequence_number);
}

extern "C" GRIFFINSIM_EXTENSION_API int GriffinSim_DequeuePwmEvent(GriffinSimPwmEvent* out_event) {
  if (out_event == nullptr) {
    return 0;
  }
  auto& state = State();
  std::lock_guard<std::mutex> lock(state.mutex);
  if (state.size == 0) {
    return 0;
  }
  *out_event = state.queue[state.head];
  state.head = (state.head + 1) % kStorageCapacity;
  state.size -= 1;
  return 1;
}

extern "C" GRIFFINSIM_EXTENSION_API void GriffinSim_ResetPwmQueue(void) {
  auto& state = State();
  std::lock_guard<std::mutex> lock(state.mutex);
  ResetQueueLocked(state);
}

extern "C" GRIFFINSIM_EXTENSION_API int GriffinSim_QueueCapacity(void) {
  auto& state = State();
  std::lock_guard<std::mutex> lock(state.mutex);
  return EffectiveCapacity(state);
}

extern "C" GRIFFINSIM_EXTENSION_API int GriffinSim_QueueSize(void) {
  auto& state = State();
  std::lock_guard<std::mutex> lock(state.mutex);
  return state.size;
}

extern "C" GRIFFINSIM_EXTENSION_API int GriffinSim_RegisterPwmSpeedCallbacks(int channel_count) {
#ifdef GRIFFINSIM_ENABLE_WPILIB_HALSIM
  if (channel_count < 0) {
    return 0;
  }

  auto& state = State();
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    if (!state.initialized) {
      return 0;
    }
  }

  const int limit = channel_count > kMaxCallbackChannels ? kMaxCallbackChannels : channel_count;
  int registered = 0;
  for (int i = 0; i < limit; ++i) {
    bool already_active = false;
    {
      std::lock_guard<std::mutex> lock(state.mutex);
      if (!state.initialized) {
        return registered;
      }
      already_active = state.callbacks[i].active;
      if (!already_active) {
        state.callbacks[i].channel = i;
      }
    }

    if (already_active) {
      ++registered;
      continue;
    }

    const int uid =
        HALSIM_RegisterPWMSpeedCallback(i, GriffinSimPwmNotifyCallback, &state.callbacks[i].channel, true);
    {
      std::lock_guard<std::mutex> lock(state.mutex);
      if (!state.initialized) {
        HALSIM_CancelPWMSpeedCallback(i, uid);
        return registered;
      }
      state.callbacks[i].uid = uid;
      state.callbacks[i].active = true;
    }
    ++registered;
  }
  return registered;
#else
  (void)channel_count;
  return 0;
#endif
}

extern "C" GRIFFINSIM_EXTENSION_API void GriffinSim_UnregisterPwmSpeedCallbacks(void) {
  auto& state = State();
  std::array<CallbackRegistration, kMaxCallbackChannels> registrations{};
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    registrations = state.callbacks;
    for (auto& registration : state.callbacks) {
      registration = CallbackRegistration{};
    }
  }
#ifdef GRIFFINSIM_ENABLE_WPILIB_HALSIM
  for (const auto& registration : registrations) {
    if (!registration.active) {
      continue;
    }
    HALSIM_CancelPWMSpeedCallback(registration.channel, registration.uid);
  }
#else
  (void)registrations;
#endif
}

extern "C" GRIFFINSIM_EXTENSION_API int GriffinSim_RegisteredPwmCallbackCount(void) {
  auto& state = State();
  std::lock_guard<std::mutex> lock(state.mutex);
  int count = 0;
  for (const auto& registration : state.callbacks) {
    if (registration.active) {
      ++count;
    }
  }
  return count;
}
