#pragma once

#include <sdkconfig.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/idf_additions.h>

// For tasks whose code paths are safe with a PSRAM stack. In particular,
// flash/NVS operations that disable the cache must run on an internal stack.
namespace task_memory {

enum class StackLocation { Failed, Internal, Psram };

namespace detail {
template <typename T, void (T::*Run)(), bool External>
void Entry(void* arg) {
    (static_cast<T*>(arg)->*Run)();
#if configSUPPORT_STATIC_ALLOCATION
    if constexpr (External) {
        // IDF owns the allocated stack and TCB; match the WithCaps creator.
        vTaskDeleteWithCaps(nullptr);
    } else
#endif
    {
        vTaskDelete(nullptr);
    }
}
}  // namespace detail

// Prefer external stack memory without changing stack size, priority or
// affinity. FreeRTOS keeps the TCB in internal RAM. The normal dynamic task
// is the fallback, including targets with no supported PSRAM stack.
template <typename T, void (T::*Run)()>
StackLocation CreatePreferPsram(const char* name, configSTACK_DEPTH_TYPE stack_bytes,
                                T* context, UBaseType_t priority, TaskHandle_t* handle,
                                bool prefer_psram = true) {
#if configSUPPORT_STATIC_ALLOCATION && CONFIG_SPIRAM && CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM
    if (prefer_psram &&
        xTaskCreateWithCaps(detail::Entry<T, Run, true>, name, stack_bytes, context,
                            priority, handle, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) == pdPASS) {
        return StackLocation::Psram;
    }
#else
    (void)prefer_psram;
#endif
    if (xTaskCreate(detail::Entry<T, Run, false>, name, stack_bytes, context,
                    priority, handle) == pdPASS) {
        return StackLocation::Internal;
    }
    return StackLocation::Failed;
}

}  // namespace task_memory
