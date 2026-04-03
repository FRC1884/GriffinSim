#pragma once

#include <stdint.h>

#ifdef _WIN32
  #ifdef GRIFFINSIM_EXTENSION_EXPORTS
    #define GRIFFINSIM_EXTENSION_API __declspec(dllexport)
  #else
    #define GRIFFINSIM_EXTENSION_API __declspec(dllimport)
  #endif
#else
  #define GRIFFINSIM_EXTENSION_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct GriffinSimPwmEvent {
  int channel;
  double value;
  uint64_t sequence_number;
} GriffinSimPwmEvent;

typedef struct GriffinSimExtensionConfig {
  int queue_capacity;
  int lockstep_enabled;
  double control_step_seconds;
  double physics_step_seconds;
} GriffinSimExtensionConfig;

GRIFFINSIM_EXTENSION_API int HALSIM_InitExtension(void);
GRIFFINSIM_EXTENSION_API void HALSIM_ShutdownExtension(void);
GRIFFINSIM_EXTENSION_API const char* GriffinSim_HalSimExtensionName(void);

GRIFFINSIM_EXTENSION_API int GriffinSim_SetExtensionConfig(const GriffinSimExtensionConfig* config);
GRIFFINSIM_EXTENSION_API int GriffinSim_GetExtensionConfig(GriffinSimExtensionConfig* config_out);

GRIFFINSIM_EXTENSION_API int GriffinSim_EnqueuePwmEvent(int channel, double value, uint64_t sequence_number);
GRIFFINSIM_EXTENSION_API int GriffinSim_DequeuePwmEvent(GriffinSimPwmEvent* out_event);
GRIFFINSIM_EXTENSION_API void GriffinSim_ResetPwmQueue(void);
GRIFFINSIM_EXTENSION_API int GriffinSim_QueueCapacity(void);
GRIFFINSIM_EXTENSION_API int GriffinSim_QueueSize(void);

GRIFFINSIM_EXTENSION_API int GriffinSim_RegisterPwmSpeedCallbacks(int channel_count);
GRIFFINSIM_EXTENSION_API void GriffinSim_UnregisterPwmSpeedCallbacks(void);
GRIFFINSIM_EXTENSION_API int GriffinSim_RegisteredPwmCallbackCount(void);

#ifdef __cplusplus
}
#endif
