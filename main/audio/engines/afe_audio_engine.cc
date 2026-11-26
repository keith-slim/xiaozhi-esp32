#include "afe_audio_engine.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <string_view>
#include <vector>

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_vadn_models.h>

#include "audio_service.h"
#include "wake_net_threshold.h"
#include "conversation_audio_settings.h"
#include <nvs.h>
#include "wake_words/custom_wake_word.h"

#define TAG "AfeAudioEngine"

extern "C" void stackchan_nvs_commit_observer(void) __attribute__((weak));

namespace WakeNetThreshold {
namespace {
constexpr char kNamespace[] = "audio";
constexpr char kKey[] = "wake_threshold";
std::atomic<int> desired{-1};
}

int Get() {
    int value = desired.load();
    if (value >= 0) return value;
    int32_t stored = kDefault;
    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READONLY, &handle) == ESP_OK) {
        if (nvs_get_i32(handle, kKey, &stored) != ESP_OK) stored = kDefault;
        nvs_close(handle);
    }
    value = IsValid(stored) ? stored : kDefault;
    int uninitialized = -1;
    // Do not overwrite a Web save that completed while NVS was being read.
    if (!desired.compare_exchange_strong(uninitialized, value)) return uninitialized;
    return value;
}

bool Save(int value) {
    if (!IsValid(value)) return false;
    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READWRITE, &handle) != ESP_OK) return false;
    esp_err_t err = nvs_set_i32(handle, kKey, value);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WakeNet threshold save failed: %s", esp_err_to_name(err));
        return false;
    }
    desired.store(value);
    if (stackchan_nvs_commit_observer) stackchan_nvs_commit_observer();
    return true;
}
} // namespace WakeNetThreshold

namespace ConversationAudio {
namespace {
constexpr char kNamespace[] = "audio";
constexpr char kKey[] = "conv_input";
std::atomic<int> desired{-1};
}

Settings Get() {
    int value = desired.load();
    if (value >= 0) return Unpack(value);
    int32_t stored = Pack(Settings{});
    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READONLY, &handle) == ESP_OK) {
        if (nvs_get_i32(handle, kKey, &stored) != ESP_OK) stored = Pack(Settings{});
        nvs_close(handle);
    }
    const Settings decoded = Unpack(stored);
    value = stored >= 0 && Valid(decoded) && Pack(decoded) == stored
        ? stored : Pack(Settings{});
    int uninitialized = -1;
    if (!desired.compare_exchange_strong(uninitialized, value)) value = uninitialized;
    return Unpack(value);
}

bool Save(const Settings& value) {
    if (!Valid(value)) return false;
    const int packed = Pack(value);
    if (Pack(Get()) == packed) return true;
    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READWRITE, &handle) != ESP_OK) return false;
    esp_err_t err = nvs_set_i32(handle, kKey, packed);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) return false;
    desired.store(packed);
    if (stackchan_nvs_commit_observer) stackchan_nvs_commit_observer();
    return true;
}

} // namespace ConversationAudio

#if CONFIG_USE_AUDIO_PROCESSOR
static constexpr bool kUseAfeForVoiceProcessing = true;
#else
static constexpr bool kUseAfeForVoiceProcessing = false;
#endif

AfeAudioEngine::AfeAudioEngine() { event_group_ = xEventGroupCreate(); }

AfeAudioEngine::~AfeAudioEngine() {
    custom_wake_word_.reset();
    if (afe_data_ != nullptr) {
        afe_iface_->destroy(afe_data_);
    }
    if (processing_task_stack_ != nullptr) {
        heap_caps_free(processing_task_stack_);
    }
    if (processing_task_buffer_ != nullptr) {
        heap_caps_free(processing_task_buffer_);
    }
    if (wake_word_encode_task_stack_ != nullptr) {
        heap_caps_free(wake_word_encode_task_stack_);
    }
    if (wake_word_encode_task_buffer_ != nullptr) {
        heap_caps_free(wake_word_encode_task_buffer_);
    }
    if (owns_models_ && models_ != nullptr) {
        esp_srmodel_deinit(models_);
    }
    if (event_group_ != nullptr) {
        vEventGroupDelete(event_group_);
    }
}

bool AfeAudioEngine::Initialize(AudioCodec* codec, int frame_duration_ms,
                                srmodel_list_t* models_list) {
    if (afe_data_ != nullptr || (codec_ != nullptr && !kUseAfeForVoiceProcessing &&
                                 wake_detector_ == WakeDetector::kNone)) {
        return true;
    }
    if (codec == nullptr) {
        ESP_LOGE(TAG, "Codec is null");
        return false;
    }

    codec_ = codec;
    // Load NVS before starting either real-time audio task.
    ConversationAudio::Get();
    frame_samples_ = frame_duration_ms * 16000 / 1000;
    output_buffer_.reserve(frame_samples_);

    if (models_list == nullptr) {
        models_ = esp_srmodel_init("model");
        owns_models_ = models_ != nullptr;
    } else {
        models_ = models_list;
    }

    char* wakenet_model_name = nullptr;
    char* multinet_model_name = nullptr;
    if (models_ != nullptr && models_->num > 0) {
        wakenet_model_name = esp_srmodel_filter(models_, ESP_WN_PREFIX, nullptr);
        multinet_model_name = esp_srmodel_filter(models_, ESP_MN_PREFIX, nullptr);
        for (int i = 0; i < models_->num; ++i) {
            ESP_LOGI(TAG, "Model %d: %s", i, models_->model_name[i]);
        }
    }

    if (multinet_model_name != nullptr) {
        wake_detector_ = WakeDetector::kMultiNet;
        custom_wake_word_ = std::make_unique<CustomWakeWord>();
        custom_wake_word_->OnWakeWordDetected([this](const std::string& wake_word) {
            last_detected_wake_word_ = wake_word;
            xEventGroupClearBits(event_group_, kWakeWordEnabled);
            UpdateActiveState();
            if (wake_word_detected_callback_) {
                wake_word_detected_callback_(wake_word);
            }
        });
        if (!custom_wake_word_->Initialize(codec_, models_)) {
            ESP_LOGE(TAG, "Failed to initialize MultiNet wake-word detector");
            custom_wake_word_.reset();
            wake_detector_ = WakeDetector::kNone;
            return false;
        }
    } else if (wakenet_model_name != nullptr) {
        wake_detector_ = WakeDetector::kWakeNet;
#if CONFIG_SEND_WAKE_WORD_DATA
        if (!wake_word_audio_cache_.Initialize(16000 * 2)) {
            ESP_LOGW(TAG, "Wake-word audio upload disabled: PSRAM cache allocation failed");
        }
#endif
    }

    const bool needs_afe = kUseAfeForVoiceProcessing || wake_detector_ != WakeDetector::kNone;
    if (!needs_afe) {
        ESP_LOGI(TAG, "Initialized as raw engine because AFE features are disabled");
        return true;
    }

    const int ref_num = codec_->input_reference() ? 1 : 0;
    const int afe_input_channels = codec_->afe_input_channels();
    if (afe_input_channels <= ref_num || afe_input_channels > codec_->input_channels()) {
        ESP_LOGE(TAG, "Invalid AFE channel count: codec=%d afe=%d ref=%d",
                 codec_->input_channels(), afe_input_channels, ref_num);
        return false;
    }
    std::string input_format;
    for (int i = 0; i < afe_input_channels - ref_num; ++i) {
        input_format.push_back('M');
    }
    for (int i = 0; i < ref_num; ++i) {
        input_format.push_back('R');
    }
    ESP_LOGI(TAG, "AFE input format: %s (codec channels: %d)", input_format.c_str(),
             codec_->input_channels());

    char* vad_model_name =
        models_ == nullptr ? nullptr : esp_srmodel_filter(models_, ESP_VADN_PREFIX, nullptr);
    afe_config_t* afe_config =
        afe_config_init(input_format.c_str(), models_, AFE_TYPE_FD, AFE_MODE_LOW_COST);
    if (afe_config == nullptr) {
        ESP_LOGE(TAG, "Failed to create AFE configuration");
        return false;
    }

    afe_config->aec_init = codec_->input_reference();
    afe_config->aec_mode = AEC_MODE_FD_LOW_COST;
    afe_config->aec_nlp_level = AEC_NLP_LEVEL_VERYAGGR;
    afe_config->ns_init = false;
    afe_config->vad_init = kUseAfeForVoiceProcessing;
    afe_config->vad_mode = VAD_MODE_0;
    afe_config->vad_min_noise_ms = 100;
    if (vad_model_name != nullptr) {
        afe_config->vad_model_name = vad_model_name;
    }
    afe_config->wakenet_init = wake_detector_ == WakeDetector::kWakeNet;
    afe_config->wakenet_model_name = nullptr;
    afe_config->wakenet_model_name_2 = nullptr;
    if (wake_detector_ == WakeDetector::kWakeNet && models_ != nullptr) {
        std::vector<char*> wakenet_models;
        for (int i = 0; i < models_->num; ++i) {
            char* name = models_->model_name[i];
            if (name != nullptr && strncmp(name, ESP_WN_PREFIX, strlen(ESP_WN_PREFIX)) == 0) {
                wakenet_models.push_back(name);
            }
        }
        if (!wakenet_models.empty()) {
            afe_config->wakenet_model_name = wakenet_models[0];
        }
        if (wakenet_models.size() > 1) {
            afe_config->wakenet_model_name_2 = wakenet_models[1];
        }
        wake_model_count_ = (afe_config->wakenet_model_name ? 1 : 0) +
                            (afe_config->wakenet_model_name_2 ? 1 : 0);
        // Only the two models selected above are active in the AFE.
        wake_words_.clear();
        for (auto name : {afe_config->wakenet_model_name, afe_config->wakenet_model_name_2}) {
            if (name == nullptr) {
                continue;
            }
            auto words = esp_srmodel_get_wake_words(models_, name);
            if (words == nullptr) {
                continue;
            }
            std::string_view remaining(words);
            while (!remaining.empty()) {
                const auto end = remaining.find(';');
                const auto word = remaining.substr(0, end);
                if (!word.empty()) {
                    wake_words_.emplace_back(word);
                }
                if (end == std::string_view::npos) {
                    break;
                }
                remaining.remove_prefix(end + 1);
            }
        }
    }
    afe_config->agc_init = false;
    afe_config->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;

    ESP_LOGI(TAG, "Before AFE create: free=%u min=%u largest=%u",
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
             heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    afe_iface_ = esp_afe_handle_from_config(afe_config);
    if (afe_iface_ != nullptr) {
        afe_data_ = afe_iface_->create_from_config(afe_config);
    }
    afe_config_free(afe_config);

    if (afe_iface_ == nullptr || afe_data_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create FD AFE instance");
        afe_iface_ = nullptr;
        afe_data_ = nullptr;
        return false;
    }

    if (wake_detector_ == WakeDetector::kWakeNet) {
        // Initialize the cached NVS setting before the real-time fetch task.
        WakeNetThreshold::Get();
        applied_wake_threshold_ = WakeNetThreshold::kDefault;
        attempted_wake_threshold_ = -1;
        afe_iface_->disable_wakenet(afe_data_);
    }
    if (codec_->input_reference()) {
        afe_iface_->disable_aec(afe_data_);
    }
    afe_iface_->print_pipeline(afe_data_);

    if (processing_task_stack_ == nullptr) {
        processing_task_stack_ = static_cast<StackType_t*>(
            heap_caps_malloc(kProcessingTaskStackSize, MALLOC_CAP_SPIRAM));
    }
    if (processing_task_buffer_ == nullptr) {
        processing_task_buffer_ =
            static_cast<StaticTask_t*>(heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL));
    }
    if (processing_task_stack_ == nullptr || processing_task_buffer_ == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate AFE processing task memory");
        afe_iface_->destroy(afe_data_);
        afe_data_ = nullptr;
        afe_iface_ = nullptr;
        return false;
    }

    // Keep AFE fetch off CPU 0 (Wi-Fi and audio_input). On CPU 1, run
    // above the priority-3 LVGL/StackChan update tasks without changing
    // the PSRAM stack, AFE pipeline, or audio input task.
    processing_task_ = xTaskCreateStaticPinnedToCore(
        [](void* arg) {
            auto* engine = static_cast<AfeAudioEngine*>(arg);
            engine->ProcessingTask();
            vTaskDelete(nullptr);
        },
        "audio_afe", kProcessingTaskStackSize, this, 4, processing_task_stack_,
        processing_task_buffer_, 1);
    if (processing_task_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create AFE processing task, internal free=%u largest=%u",
                 heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        afe_iface_->destroy(afe_data_);
        afe_data_ = nullptr;
        afe_iface_ = nullptr;
        return false;
    }

    const char* detector = wake_detector_ == WakeDetector::kWakeNet
                               ? "WakeNet"
                               : (wake_detector_ == WakeDetector::kMultiNet ? "MultiNet" : "none");
    ESP_LOGI(TAG, "Initialized FD AFE, detector: %s, NS: off, feed: %d, fetch: %d", detector,
             afe_iface_->get_feed_chunksize(afe_data_), afe_iface_->get_fetch_chunksize(afe_data_));
    ESP_LOGI(TAG, "After AFE create: free=%u min=%u largest=%u",
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
             heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    return true;
}

void AfeAudioEngine::Feed(std::vector<int16_t>&& data) {
    EventBits_t bits = xEventGroupGetBits(event_group_);
    if ((bits & kVoiceProcessingEnabled) && !kUseAfeForVoiceProcessing) {
        OutputRawAudio(data);
    }
    if (afe_data_ == nullptr || (bits & kAfeActive) == 0) {
        return;
    }

    std::lock_guard<std::mutex> lock(input_buffer_mutex_);
    if ((xEventGroupGetBits(event_group_) & kAfeActive) == 0) {
        return;
    }
    const size_t codec_channels = codec_->input_channels();
    const size_t afe_channels = codec_->afe_input_channels();
    if (codec_channels == afe_channels) {
        input_buffer_.insert(input_buffer_.end(), data.begin(), data.end());
    } else {
        // 生PCMの追加マイクは方向推定用に保持し、AFEには必要なM...Rだけを詰め直す。
        const size_t reference_channels = codec_->input_reference() ? 1u : 0u;
        const size_t afe_microphone_channels = afe_channels - reference_channels;
        const size_t frames = data.size() / codec_channels;
        input_buffer_.reserve(input_buffer_.size() + frames * afe_channels);
        for (size_t frame = 0; frame < frames; ++frame) {
            const size_t offset = frame * codec_channels;
            for (size_t channel = 0; channel < afe_microphone_channels; ++channel) {
                input_buffer_.push_back(data[offset + channel]);
            }
            if (reference_channels != 0) {
                input_buffer_.push_back(data[offset + codec_channels - 1]);
            }
        }
    }
    size_t chunk_size = afe_iface_->get_feed_chunksize(afe_data_) * afe_channels;
    while (input_buffer_.size() >= chunk_size) {
        afe_iface_->feed(afe_data_, input_buffer_.data());
        input_buffer_.erase(input_buffer_.begin(), input_buffer_.begin() + chunk_size);
    }
}

void AfeAudioEngine::EnableWakeWordDetection(bool enable) {
    if (!HasWakeWord()) {
        return;
    }

    // WakeNet enable/disable on the AFE instance is applied by ProcessingTask
    // (see ApplyAfeControls), driven by the kWakeWordEnabled bit.
    if (enable) {
        if (wake_detector_ == WakeDetector::kMultiNet) {
            custom_wake_word_->Start();
        }
        xEventGroupSetBits(event_group_, kWakeWordEnabled);
    } else {
        xEventGroupClearBits(event_group_, kWakeWordEnabled);
        if (wake_detector_ == WakeDetector::kMultiNet) {
            custom_wake_word_->Stop();
        }
    }
    UpdateActiveState();
}

void AfeAudioEngine::EnableVoiceProcessing(bool enable) {
    if (enable) {
        xEventGroupSetBits(event_group_, kVoiceProcessingEnabled);
    } else {
        xEventGroupClearBits(event_group_, kVoiceProcessingEnabled);
        is_speaking_ = false;
    }
    UpdateActiveState();
}

void AfeAudioEngine::EnableDeviceAec(bool enable) {
    device_aec_enabled_.store(enable);
    if (enable && (codec_ == nullptr || !codec_->input_reference())) {
        ESP_LOGW(TAG, "Device AEC requires a playback reference channel");
    }
    UpdateAecState();
}

bool AfeAudioEngine::HasWakeWord() const { return wake_detector_ != WakeDetector::kNone; }

std::vector<std::string> AfeAudioEngine::GetWakeWords() const {
    return custom_wake_word_ ? custom_wake_word_->GetWakeWords() : wake_words_;
}

bool AfeAudioEngine::IsWakeWordDetectionEnabled() const {
    return event_group_ != nullptr && (xEventGroupGetBits(event_group_) & kWakeWordEnabled) != 0;
}

bool AfeAudioEngine::IsVoiceProcessingEnabled() const {
    return event_group_ != nullptr &&
           (xEventGroupGetBits(event_group_) & kVoiceProcessingEnabled) != 0;
}

size_t AfeAudioEngine::GetFeedSize() const {
    return afe_data_ == nullptr ? frame_samples_ : afe_iface_->get_feed_chunksize(afe_data_);
}

void AfeAudioEngine::OnWakeWordDetected(
    std::function<void(const std::string& wake_word)> callback) {
    wake_word_detected_callback_ = std::move(callback);
}

void AfeAudioEngine::OnOutput(std::function<void(std::vector<int16_t>&& data)> callback) {
    output_callback_ = std::move(callback);
}

void AfeAudioEngine::OnVadStateChange(std::function<void(bool speaking)> callback) {
    vad_state_change_callback_ = std::move(callback);
}

void AfeAudioEngine::UpdateActiveState() {
    EventBits_t bits = xEventGroupGetBits(event_group_);
    const bool afe_active =
        afe_data_ != nullptr && ((bits & kWakeWordEnabled) ||
                                 (kUseAfeForVoiceProcessing && (bits & kVoiceProcessingEnabled)));
    if (afe_active) {
        xEventGroupSetBits(event_group_, kAfeActive);
    } else {
        xEventGroupClearBits(event_group_, kAfeActive);
        control_generation_.fetch_add(1);
        std::lock_guard<std::mutex> lock(input_buffer_mutex_);
        input_buffer_.clear();
        if (afe_data_ != nullptr) {
            // Don't call reset_buffer() here: this runs in the main task while
            // ProcessingTask may be inside fetch_with_delay() on the same AFE
            // instance, and a concurrent reset corrupts the ring buffer state.
            // Defer the reset to ProcessingTask, which owns the fetch side.
            reset_pending_ = true;
        }
    }
    if ((bits & kVoiceProcessingEnabled) == 0) {
        // output_buffer_ is owned by the task that produces output frames,
        // so let that task clear it instead of racing with it here.
        output_reset_pending_ = true;
    }
    afe_control_dirty_ = true;
}

void AfeAudioEngine::UpdateAecState() {
    if (afe_data_ == nullptr || codec_ == nullptr || !codec_->input_reference()) {
        return;
    }
    afe_control_dirty_ = true;
}

void AfeAudioEngine::ApplyAfeControls() {
    EventBits_t bits = xEventGroupGetBits(event_group_);
    if (wake_detector_ == WakeDetector::kWakeNet) {
        if (bits & kWakeWordEnabled) {
            afe_iface_->enable_wakenet(afe_data_);
        } else {
            afe_iface_->disable_wakenet(afe_data_);
        }
    }
    if (codec_->input_reference()) {
        const bool enable_aec = (bits & kWakeWordEnabled) ||
                                (device_aec_enabled_.load() && (bits & kVoiceProcessingEnabled));
        if (enable_aec) {
            afe_iface_->enable_aec(afe_data_);
        } else {
            afe_iface_->disable_aec(afe_data_);
        }
    }
}

void AfeAudioEngine::ApplyWakeNetThreshold() {
    if (wake_detector_ != WakeDetector::kWakeNet ||
        !(xEventGroupGetBits(event_group_) & kWakeWordEnabled)) return;
    const int value = WakeNetThreshold::Get();
    if (value == attempted_wake_threshold_) return;
    attempted_wake_threshold_ = value;
    // Preserve the original model defaults without calling the setter on a
    // fresh AFE. A transition back to default uses the official reset API.
    if (value == WakeNetThreshold::kDefault &&
        applied_wake_threshold_ == WakeNetThreshold::kDefault) {
        ESP_LOGI(TAG, "WakeNet threshold: model default");
        return;
    }
    bool ok = wake_model_count_ > 0;
    for (int index = 1; index <= wake_model_count_; ++index) {
        const int rc = value == WakeNetThreshold::kDefault
            ? afe_iface_->reset_wakenet_threshold(afe_data_, index)
            : afe_iface_->set_wakenet_threshold(afe_data_, index, value / 100.0f);
        if (rc != 1) ok = false;
        ESP_LOGI(TAG, "WakeNet threshold: model=%d setting=%d result=%d", index, value, rc);
    }
    // A partial failure must not be mistaken for model defaults later.
    applied_wake_threshold_ = ok ? value : -1;
    if (!ok) ESP_LOGW(TAG, "WakeNet threshold apply failed; change setting to retry");
}

void AfeAudioEngine::ApplyPendingReset() {
    if (!reset_pending_.exchange(false)) {
        return;
    }
    // Discard audio recorded before (re)activation. Holding input_buffer_mutex_
    // serializes the reset against Feed(); fetch/reset both run in this task.
    std::lock_guard<std::mutex> lock(input_buffer_mutex_);
    input_buffer_.clear();
    afe_iface_->reset_buffer(afe_data_);
}

void AfeAudioEngine::ProcessingTask() {
    // Per-task counters; no allocations or logging in the per-frame hot path.
    uint64_t fetch_total_us = 0;
    uint64_t process_total_us = 0;
    uint32_t fetch_max_us = 0;
    uint32_t process_max_us = 0;
    uint32_t fetch_frames = 0;
    uint32_t fetch_errors = 0;
    int64_t timing_last_log_us = esp_timer_get_time();
    auto report_afe_timing = [&]() {
        const int64_t now = esp_timer_get_time();
        if (now - timing_last_log_us < 10000000) return;
        ESP_LOGI(TAG,
                 "[AUDIO_TIMING] afe fetch_avg=%luus fetch_max=%luus "
                 "process_avg=%luus process_max=%luus frames=%lu errors=%lu /10s",
                 static_cast<unsigned long>(fetch_frames ? fetch_total_us / fetch_frames : 0),
                 static_cast<unsigned long>(fetch_max_us),
                 static_cast<unsigned long>(fetch_frames ? process_total_us / fetch_frames : 0),
                 static_cast<unsigned long>(process_max_us),
                 static_cast<unsigned long>(fetch_frames),
                 static_cast<unsigned long>(fetch_errors));
#if (INCLUDE_uxTaskGetStackHighWaterMark == 1)
        ESP_LOGI(TAG, "[STACK_DIAG] audio_afe_free=%u",
                 (unsigned)uxTaskGetStackHighWaterMark(nullptr));
#endif
        fetch_total_us = process_total_us = 0;
        fetch_max_us = process_max_us = fetch_frames = fetch_errors = 0;
        timing_last_log_us = now;
    };

    while (true) {
        xEventGroupWaitBits(event_group_, kAfeActive, pdFALSE, pdTRUE, portMAX_DELAY);
        ApplyPendingReset();
        if ((xEventGroupGetBits(event_group_) & kAfeActive) == 0) {
            continue;
        }
        if (afe_control_dirty_.exchange(false)) {
            // WakeNet/AEC toggles are not safe against a concurrent fetch,
            // so they are applied here, in the task that owns the fetch side.
            ApplyAfeControls();
        }
        // Only the fetch owner changes WakeNet settings, never the HTTP task.
        ApplyWakeNetThreshold();
        const uint32_t generation = control_generation_.load();
        const int64_t fetch_begin = esp_timer_get_time();
        auto* result = afe_iface_->fetch_with_delay(afe_data_, portMAX_DELAY);
        const int64_t fetch_end = esp_timer_get_time();
        if (generation != control_generation_.load() ||
            (xEventGroupGetBits(event_group_) & kAfeActive) == 0) {
            // A disable/re-enable may make an old blocked fetch return after the
            // AFE is active again. Reset immediately and never process that frame.
            ApplyPendingReset();
            continue;
        }
        if (result == nullptr || result->ret_value == ESP_FAIL) {
            ++fetch_errors;
            if (result != nullptr) {
                ESP_LOGW(TAG, "AFE fetch failed: %d", result->ret_value);
            }
            report_afe_timing();
            continue;
        }

        EventBits_t bits = xEventGroupGetBits(event_group_);
        if (bits & kWakeWordEnabled) {
            HandleWakeWordResult(result);
        }
        if (kUseAfeForVoiceProcessing && (bits & kVoiceProcessingEnabled)) {
            HandleVoiceResult(result);
        }
        const int64_t process_end = esp_timer_get_time();
        const auto fetch_us = static_cast<uint32_t>(fetch_end - fetch_begin);
        const auto process_us = static_cast<uint32_t>(process_end - fetch_end);
        fetch_total_us += fetch_us;
        process_total_us += process_us;
        fetch_max_us = std::max(fetch_max_us, fetch_us);
        process_max_us = std::max(process_max_us, process_us);
        ++fetch_frames;
        report_afe_timing();
    }
}

void AfeAudioEngine::HandleWakeWordResult(const afe_fetch_result_t* result) {
    if (wake_detector_ == WakeDetector::kMultiNet) {
        custom_wake_word_->FeedMono(result->data, result->data_size / sizeof(int16_t));
        return;
    }

#if CONFIG_SEND_WAKE_WORD_DATA
    wake_word_audio_cache_.Store(result->data, result->data_size / sizeof(int16_t));
#endif
    if (result->wakeup_state != WAKENET_DETECTED) {
        return;
    }

    int model_index = result->wakenet_model_index - 1;
    if (model_index < 0 || model_index >= static_cast<int>(wake_words_.size())) {
        ESP_LOGE(TAG, "Invalid WakeNet model index: %d", result->wakenet_model_index);
        return;
    }

    last_detected_wake_word_ = wake_words_[model_index];
    xEventGroupClearBits(event_group_, kWakeWordEnabled);
    // UpdateActiveState marks the AFE controls dirty; the next loop iteration
    // of ProcessingTask disables WakeNet via ApplyAfeControls.
    UpdateActiveState();
    if (wake_word_detected_callback_) {
        wake_word_detected_callback_(last_detected_wake_word_);
    }
}

// Only the output sent to the conversation/Opus encoder is adjusted.
// WakeNet/MultiNet, microphone DOA, the AEC reference, and the shared
// AFE's internal AGC pipeline are untouched. No additional frame buffers.
void AfeAudioEngine::AppendConversationAudio(const int16_t* data, size_t samples, size_t stride) {
    if (!data || samples == 0 || stride == 0) return;
    const auto settings = ConversationAudio::Get();
    const int packed = ConversationAudio::Pack(settings);
    if (packed != conversation_settings_) {
        conversation_settings_ = packed;
        conversation_manual_gain_ = std::pow(10.0f, settings.gain_db / 20.0f);
        conversation_agc_limit_ = std::pow(10.0f, settings.agc * 3.0f / 20.0f);
        conversation_agc_gain_ = 1.0f;
        ESP_LOGI(TAG, "[CONV_AUDIO] gain_db=%d agc=%d gemini_vad_requested=%d",
                 settings.gain_db, settings.agc, settings.vad_mode);
    }
    if (settings.gain_db == 0 && settings.agc == 0) {
        // Exact original PCM path for default settings.
        if (stride == 1) output_buffer_.insert(output_buffer_.end(), data, data + samples);
        else for (size_t i = 0; i < samples; ++i) output_buffer_.push_back(data[i * stride]);
        return;
    }

    float gain = conversation_manual_gain_;
    if (settings.agc != 0) {
        uint64_t sum = 0;
        int peak = 0;
        for (size_t i = 0; i < samples; ++i) {
            const int sample = data[i * stride];
            sum += static_cast<uint64_t>(sample * sample);
            peak = std::max(peak, sample < 0 ? -sample : sample);
        }
        const float rms = std::sqrt(static_cast<float>(sum) / samples) * gain;
        // Do not raise near-silent input; cap the extra gain to the selected
        // strength. Faster attack for loud sounds, slower gain recovery.
        const float target = rms >= 128.0f
            ? std::clamp(4096.0f / rms, 1.0f, conversation_agc_limit_) : 1.0f;
        conversation_agc_gain_ += (target - conversation_agc_gain_) *
            (target < conversation_agc_gain_ ? 0.5f : 0.1f);
        gain *= conversation_agc_gain_;
        if (peak > 0) gain = std::min(gain, 32767.0f / peak);
    }
    for (size_t i = 0; i < samples; ++i) {
        const float scaled = data[i * stride] * gain;
        output_buffer_.push_back(static_cast<int16_t>(std::clamp(scaled, -32768.0f, 32767.0f)));
    }
}

void AfeAudioEngine::HandleVoiceResult(const afe_fetch_result_t* result) {
    if (output_reset_pending_.exchange(false)) {
        output_buffer_.clear();
        conversation_settings_ = -1;
    }
    if (vad_state_change_callback_) {
        if (result->vad_state == VAD_SPEECH && !is_speaking_) {
            is_speaking_ = true;
            vad_state_change_callback_(true);
        } else if (result->vad_state == VAD_SILENCE && is_speaking_) {
            is_speaking_ = false;
            vad_state_change_callback_(false);
        }
    }
    if (!output_callback_) {
        return;
    }

    size_t samples = result->data_size / sizeof(int16_t);
    AppendConversationAudio(result->data, samples, 1);
    while (output_buffer_.size() >= static_cast<size_t>(frame_samples_)) {
        if (output_buffer_.size() == static_cast<size_t>(frame_samples_)) {
            output_callback_(std::move(output_buffer_));
            output_buffer_.clear();
            output_buffer_.reserve(frame_samples_);
        } else {
            output_callback_(std::vector<int16_t>(output_buffer_.begin(),
                                                  output_buffer_.begin() + frame_samples_));
            output_buffer_.erase(output_buffer_.begin(), output_buffer_.begin() + frame_samples_);
        }
    }
}

void AfeAudioEngine::OutputRawAudio(const std::vector<int16_t>& data) {
    if (!output_callback_ || codec_ == nullptr) {
        return;
    }
    if (output_reset_pending_.exchange(false)) {
        output_buffer_.clear();
        conversation_settings_ = -1;
    }
    const size_t channels = std::max(codec_->input_channels(), 1);
    AppendConversationAudio(data.data(), data.size() / channels, channels);
    while (output_buffer_.size() >= static_cast<size_t>(frame_samples_)) {
        output_callback_(
            std::vector<int16_t>(output_buffer_.begin(), output_buffer_.begin() + frame_samples_));
        output_buffer_.erase(output_buffer_.begin(), output_buffer_.begin() + frame_samples_);
    }
}

void AfeAudioEngine::EncodeWakeWordData() {
    if (wake_detector_ == WakeDetector::kMultiNet) {
        custom_wake_word_->EncodeWakeWordData();
        return;
    }
    if (wake_detector_ != WakeDetector::kWakeNet) {
        return;
    }

    const size_t stack_size = 4096 * 6;
    wake_word_opus_.clear();
    if (wake_word_encode_task_stack_ == nullptr) {
        wake_word_encode_task_stack_ =
            static_cast<StackType_t*>(heap_caps_malloc(stack_size, MALLOC_CAP_SPIRAM));
        assert(wake_word_encode_task_stack_ != nullptr);
    }
    if (wake_word_encode_task_buffer_ == nullptr) {
        wake_word_encode_task_buffer_ =
            static_cast<StaticTask_t*>(heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL));
        assert(wake_word_encode_task_buffer_ != nullptr);
    }

    wake_word_encode_task_ = xTaskCreateStatic(
        [](void* arg) {
            auto* engine = static_cast<AfeAudioEngine*>(arg);
            auto start_time = esp_timer_get_time();
            esp_opus_enc_config_t opus_enc_cfg = AS_OPUS_ENC_CONFIG();
            void* encoder_handle = nullptr;
            auto ret =
                esp_opus_enc_open(&opus_enc_cfg, sizeof(esp_opus_enc_config_t), &encoder_handle);
            if (encoder_handle == nullptr) {
                ESP_LOGE(TAG, "Failed to create wake-word encoder: %d", ret);
                engine->wake_word_audio_cache_.Clear();
                {
                    std::lock_guard<std::mutex> lock(engine->wake_word_mutex_);
                    engine->wake_word_opus_.emplace_back();
                    engine->wake_word_cv_.notify_all();
                }
                vTaskDelete(nullptr);
                return;
            }

            int frame_size = 0;
            int outbuf_size = 0;
            esp_opus_enc_get_frame_size(encoder_handle, &frame_size, &outbuf_size);
            frame_size /= sizeof(int16_t);
            int packets = 0;
            std::vector<int16_t> input(frame_size);
            esp_audio_enc_in_frame_t in = {};
            esp_audio_enc_out_frame_t out = {};

            const size_t cached_samples = engine->wake_word_audio_cache_.Size();
            for (size_t offset = 0; offset + static_cast<size_t>(frame_size) <= cached_samples;
                 offset += frame_size) {
                if (engine->wake_word_audio_cache_.Read(offset, input.data(), frame_size) !=
                    static_cast<size_t>(frame_size)) {
                    break;
                }
                std::vector<uint8_t> opus_buf(outbuf_size);
                in.buffer = reinterpret_cast<uint8_t*>(input.data());
                in.len = frame_size * sizeof(int16_t);
                out.buffer = opus_buf.data();
                out.len = outbuf_size;
                out.encoded_bytes = 0;
                ret = esp_opus_enc_process(encoder_handle, &in, &out);
                if (ret == ESP_AUDIO_ERR_OK) {
                    std::lock_guard<std::mutex> lock(engine->wake_word_mutex_);
                    engine->wake_word_opus_.emplace_back(opus_buf.data(),
                                                         opus_buf.data() + out.encoded_bytes);
                    engine->wake_word_cv_.notify_all();
                    ++packets;
                } else {
                    ESP_LOGE(TAG, "Failed to encode wake-word audio: %d", ret);
                }
            }

            engine->wake_word_audio_cache_.Clear();
            esp_opus_enc_close(encoder_handle);
            ESP_LOGI(TAG, "Encoded wake word into %d packets in %ld ms", packets,
                     static_cast<long>((esp_timer_get_time() - start_time) / 1000));
            {
                std::lock_guard<std::mutex> lock(engine->wake_word_mutex_);
                engine->wake_word_opus_.emplace_back();
                engine->wake_word_cv_.notify_all();
            }
            vTaskDelete(nullptr);
        },
        "encode_wake_word", stack_size, this, 2, wake_word_encode_task_stack_,
        wake_word_encode_task_buffer_);
}

bool AfeAudioEngine::GetWakeWordOpus(std::vector<uint8_t>& opus) {
    if (wake_detector_ == WakeDetector::kMultiNet) {
        return custom_wake_word_->GetWakeWordOpus(opus);
    }
    if (wake_detector_ != WakeDetector::kWakeNet) {
        return false;
    }
    std::unique_lock<std::mutex> lock(wake_word_mutex_);
    wake_word_cv_.wait(lock, [this]() { return !wake_word_opus_.empty(); });
    opus.swap(wake_word_opus_.front());
    wake_word_opus_.pop_front();
    return !opus.empty();
}
