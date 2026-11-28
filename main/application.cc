#include "application.h"
#include <hal/hal.h>
#include "assets.h"
#include "assets/lang_config.h"
#include "audio_codec.h"
#include "board.h"
#include "cjson_utils.h"
#include "display.h"
#include "mcp_server.h"
#include "mqtt_protocol.h"
#include "settings.h"
#include "system_info.h"
#include "text_glyph_payload.h"
#include "websocket_protocol.h"
#include <esp_app_desc.h>
#if CONFIG_BOARD_TYPE_M5STACK_STACK_CHAN
#include <stackchan/config/server_boot_policy.h>
#endif

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_random.h>
#include <esp_heap_caps.h>
#include <arpa/inet.h>
#include <cJSON.h>
#include <algorithm>
#include <cstring>
#include <limits>

#define TAG "Application"

// Diagnostic-only: internal heap, largest contiguous allocation and PSRAM.
static void LogMemorySnapshot(const char* stage) {
    ESP_LOGI(TAG, "[MEM_DIAG] %s internal=%u largest=%u min_internal=%u psram=%u",
             stage,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
}

Application::Application() : notify_player_(audio_service_) {
    event_group_ = xEventGroupCreate();

#if CONFIG_USE_DEVICE_AEC && CONFIG_USE_SERVER_AEC
#error "CONFIG_USE_DEVICE_AEC and CONFIG_USE_SERVER_AEC cannot be enabled at the same time"
#elif CONFIG_USE_DEVICE_AEC
    aec_mode_ = kAecOnDeviceSide;
#elif CONFIG_USE_SERVER_AEC
    aec_mode_ = kAecOnServerSide;
#else
    aec_mode_ = kAecOff;
#endif

    esp_timer_create_args_t clock_timer_args = {.callback =
                                                    [](void* arg) {
                                                        Application* app = (Application*)arg;
                                                        xEventGroupSetBits(app->event_group_,
                                                                           MAIN_EVENT_CLOCK_TICK);
                                                    },
                                                .arg = this,
                                                .dispatch_method = ESP_TIMER_TASK,
                                                .name = "clock_timer",
                                                .skip_unhandled_events = true};
    esp_timer_create(&clock_timer_args, &clock_timer_handle_);
}

Application::~Application() {
    notify_player_.Stop();
    if (clock_timer_handle_ != nullptr) {
        esp_timer_stop(clock_timer_handle_);
        esp_timer_delete(clock_timer_handle_);
    }
    if (startup_wake_word_hint_timer_ != nullptr) {
        esp_timer_stop(startup_wake_word_hint_timer_);
        esp_timer_delete(startup_wake_word_hint_timer_);
    }
    vEventGroupDelete(event_group_);
}

bool Application::SetDeviceState(DeviceState state) {
    // A text-only WebSocket announcement goes directly from CONNECTING to
    // SPEAKING. Permit that one transition without entering LISTENING or
    // enabling the microphone; ordinary conversations retain strict rules.
    const bool standalone_speech =
        state == kDeviceStateSpeaking && IsAnnouncementActive();
    return state_machine_.TransitionTo(state, standalone_speech);
}

void Application::Initialize() {
    auto& board = Board::GetInstance();
    SetDeviceState(kDeviceStateStarting);

    // Setup the display
    auto display = board.GetDisplay();
    display->SetupUI();
    // Print board name/version info
    display->SetChatMessage("system", SystemInfo::GetUserAgent().c_str());

    // Setup the audio service
    LogMemorySnapshot("before_audio_init");
    auto codec = board.GetAudioCodec();
    audio_service_.Initialize(codec);
    LogMemorySnapshot("after_audio_init");
    audio_service_.Start();
    LogMemorySnapshot("after_audio_tasks");
    ESP_LOGI(TAG, "After board/audio init");
    SystemInfo::PrintHeapStats();

    AudioServiceCallbacks callbacks;
    callbacks.on_send_queue_available = [this]() {
        xEventGroupSetBits(event_group_, MAIN_EVENT_SEND_AUDIO);
    };
    callbacks.on_wake_word_detected = [this](const std::string& wake_word) {
        xEventGroupSetBits(event_group_, MAIN_EVENT_WAKE_WORD_DETECTED);
    };
    callbacks.on_vad_change = [this](bool speaking) {
        xEventGroupSetBits(event_group_, MAIN_EVENT_VAD_CHANGE);
    };
    callbacks.on_playback_drained = [this]() {
        xEventGroupSetBits(event_group_, MAIN_EVENT_PLAYBACK_DRAINED);
    };
    callbacks.on_playback_progress = [this](uint32_t playback_id, uint32_t media_position_ms) {
        notify_player_.OnPlaybackProgress(playback_id, media_position_ms);
    };
    callbacks.on_speech_playback_progress = [this](uint32_t played_ms) {
        if (GetDeviceState() != kDeviceStateSpeaking) return;
        const uint32_t previous = last_speech_subtitle_tick_ms_.load();
        if (played_ms < previous || (played_ms - previous < 180 && previous != 0)) return;
        last_speech_subtitle_tick_ms_.store(played_ms);
        Schedule([this, played_ms]() {
            if (GetDeviceState() == kDeviceStateSpeaking) {
                const auto elapsed = std::to_string(played_ms);
                Board::GetInstance().GetDisplay()->SetChatMessage("assistant_tick", elapsed.c_str());
            }
        });
    };
    callbacks.on_speech_phrase_boundary = [this](uint32_t played_ms) {
        if (GetDeviceState() != kDeviceStateSpeaking) return;
        Schedule([this, played_ms]() {
            if (GetDeviceState() != kDeviceStateSpeaking) return;
            const auto elapsed = std::to_string(played_ms);
            Board::GetInstance().GetDisplay()->SetChatMessage("assistant_pause", elapsed.c_str());
        });
    };
    audio_service_.SetCallbacks(callbacks);

    // Add state change listeners
    state_machine_.AddStateChangeListener([this](DeviceState old_state, DeviceState new_state) {
        xEventGroupSetBits(event_group_, MAIN_EVENT_STATE_CHANGED);
        const bool conversation_state = old_state == kDeviceStateConnecting ||
                                        old_state == kDeviceStateListening ||
                                        old_state == kDeviceStateSpeaking;
        if (conversation_state && new_state == kDeviceStateIdle) {
            Board::GetInstance().OnConversationEnded();
        }
    });

    // Start the clock timer to update the status bar
    esp_timer_start_periodic(clock_timer_handle_, 1000000);

    // Add MCP common tools (only once during initialization)
    auto& mcp_server = McpServer::GetInstance();
    mcp_server.AddCommonTools();
    mcp_server.AddUserOnlyTools();
    // Registration is complete here, so this is the only place the whole inventory exists.
    // It is logged because a server that accepts a bounded number of tools drops the rest
    // quietly: the only sign on this device was an alert at boot and a sad face, which is
    // easy to see and hard to connect to a tool list.
    mcp_server.LogToolInventory();

    // Set network event callback for UI updates and network state handling
    board.SetNetworkEventCallback([this](NetworkEvent event, const std::string& data) {
        auto display = Board::GetInstance().GetDisplay();

        switch (event) {
            case NetworkEvent::Scanning:
                display->ShowNotification(Lang::Strings::SCANNING_WIFI, 30000);
                xEventGroupSetBits(event_group_, MAIN_EVENT_NETWORK_DISCONNECTED);
                break;
            case NetworkEvent::Connecting: {
                if (data.empty()) {
                    // Cellular network - registering without carrier info yet
                    display->SetStatus(Lang::Strings::REGISTERING_NETWORK);
                } else {
                    // WiFi or cellular with carrier info
                    std::string msg = Lang::Strings::CONNECT_TO;
                    msg += data;
                    msg += "...";
                    display->ShowNotification(msg.c_str(), 30000);
                }
                break;
            }
            case NetworkEvent::Connected: {
                std::string msg = Lang::Strings::CONNECTED_TO;
                msg += data;
                display->ShowNotification(msg.c_str(), 30000);
                xEventGroupSetBits(event_group_, MAIN_EVENT_NETWORK_CONNECTED);
                break;
            }
            case NetworkEvent::Disconnected:
                xEventGroupSetBits(event_group_, MAIN_EVENT_NETWORK_DISCONNECTED);
                break;
            case NetworkEvent::WifiConfigModeEnter:
                // WiFi config mode enter is handled by WifiBoard internally
                break;
            case NetworkEvent::WifiConfigModeExit:
                // WiFi config mode exit is handled by WifiBoard internally
                break;
            // Cellular modem specific events
            case NetworkEvent::ModemDetecting:
                display->SetStatus(Lang::Strings::DETECTING_MODULE);
                break;
            case NetworkEvent::ModemErrorNoSim:
                Alert(Lang::Strings::ERROR, Lang::Strings::PIN_ERROR, "warning",
                      Lang::Sounds::OGG_ERR_PIN);
                break;
            case NetworkEvent::ModemErrorRegDenied:
                Alert(Lang::Strings::ERROR, Lang::Strings::REG_ERROR, "warning",
                      Lang::Sounds::OGG_ERR_REG);
                break;
            case NetworkEvent::ModemErrorInitFailed:
                Alert(Lang::Strings::ERROR, Lang::Strings::MODEM_INIT_ERROR, "warning",
                      Lang::Sounds::OGG_EXCLAMATION);
                break;
            case NetworkEvent::ModemErrorTimeout:
                display->SetStatus(Lang::Strings::REGISTERING_NETWORK);
                break;
        }
    });

    // Start network asynchronously
    board.StartNetwork();

    // Update the status bar immediately to show the network state
    display->UpdateStatusBar(true);
}

void Application::Run() {
    // Set the priority of the main task to 10
    vTaskPrioritySet(nullptr, 10);

    const EventBits_t ALL_EVENTS =
        MAIN_EVENT_SCHEDULE | MAIN_EVENT_SEND_AUDIO | MAIN_EVENT_WAKE_WORD_DETECTED |
        MAIN_EVENT_VAD_CHANGE | MAIN_EVENT_CLOCK_TICK | MAIN_EVENT_ERROR |
        MAIN_EVENT_NETWORK_CONNECTED | MAIN_EVENT_NETWORK_DISCONNECTED | MAIN_EVENT_TOGGLE_CHAT |
        MAIN_EVENT_START_LISTENING | MAIN_EVENT_STOP_LISTENING | MAIN_EVENT_ACTIVATION_DONE |
        MAIN_EVENT_STATE_CHANGED | MAIN_EVENT_PLAYBACK_DRAINED | MAIN_EVENT_STARTUP_HINT_TIMEOUT;

    while (true) {
        auto bits = xEventGroupWaitBits(event_group_, ALL_EVENTS, pdTRUE, pdFALSE, portMAX_DELAY);

        if (bits & MAIN_EVENT_ERROR) {
            if (GetDeviceState() == kDeviceStateNotifying) {
                StopNotification();
            }
            SetDeviceState(kDeviceStateIdle);
            const std::string error = LastErrorMessage();
            Alert(Lang::Strings::ERROR, error.c_str(), "cancel", Lang::Sounds::OGG_EXCLAMATION);
        }

        if (bits & MAIN_EVENT_NETWORK_CONNECTED) {
            HandleNetworkConnectedEvent();
        }

        if (bits & MAIN_EVENT_NETWORK_DISCONNECTED) {
            HandleNetworkDisconnectedEvent();
        }

        if (bits & MAIN_EVENT_ACTIVATION_DONE) {
            HandleActivationDoneEvent();
        }

        if (bits & MAIN_EVENT_STATE_CHANGED) {
            HandleStateChangedEvent();
        }

        if (bits & MAIN_EVENT_PLAYBACK_DRAINED) {
            if (audio_service_.IsPlaybackIdle()) {
                notify_player_.OnPlaybackDrained();
                // TTS stop can arrive before decoded PCM has finished playing.
                // Do not enter listening mode early: otherwise playback ticks
                // stop and StackChan treats all remaining text as a review.
                if (tts_stop_pending_ && GetDeviceState() == kDeviceStateSpeaking) {
                    tts_stop_pending_ = false;
                    ESP_LOGI(TAG, "TTS playback drained; leaving speaking state");
                    if (IsAnnouncementActive()) FinishAnnouncement();
                    else FinishConversationPlayback();
                }
            }
            // Deferred listening start (auto mode): the playback queue has
            // drained, so it is now safe to enable voice processing.
            if (pending_listening_start_ && GetDeviceState() == kDeviceStateListening &&
                audio_service_.IsPlaybackIdle()) {
                pending_listening_start_ = false;
                StartListeningAudio();
            }
        }

        if (bits & MAIN_EVENT_TOGGLE_CHAT) {
            HandleToggleChatEvent();
        }

        if (bits & MAIN_EVENT_START_LISTENING) {
            HandleStartListeningEvent();
        }

        if (bits & MAIN_EVENT_STOP_LISTENING) {
            HandleStopListeningEvent();
        }

        if (bits & MAIN_EVENT_SEND_AUDIO) {
            while (auto packet = audio_service_.PopPacketFromSendQueue()) {
                if (protocol_ && !protocol_->SendAudio(std::move(packet))) {
                    // Drop the remaining packets. Leaving them in the queue would
                    // stall the Opus codec task (it waits for queue space), which in
                    // turn deadlocks the whole audio input pipeline, as no new
                    // MAIN_EVENT_SEND_AUDIO event would ever be triggered again.
                    while (audio_service_.PopPacketFromSendQueue())
                        ;
                    break;
                }
            }
        }

        if (bits & MAIN_EVENT_WAKE_WORD_DETECTED) {
            HandleWakeWordDetectedEvent();
        }

        if (bits & MAIN_EVENT_VAD_CHANGE) {
            if (GetDeviceState() == kDeviceStateListening) {
                auto led = Board::GetInstance().GetLed();
                led->OnStateChanged();
            }
        }

        if (bits & MAIN_EVENT_SCHEDULE) {
            std::unique_lock<std::mutex> lock(mutex_);
            auto tasks = std::move(main_tasks_);
            lock.unlock();
            for (auto& task : tasks) {
                task();
            }
        }

        if (bits & MAIN_EVENT_CLOCK_TICK) {
            clock_ticks_++;
            auto display = Board::GetInstance().GetDisplay();
            display->UpdateStatusBar();
            TryStartLocalNetworkService();
            MaybeReconnectAudioChannel();
            // A wake word can interrupt standalone speech already playing through
            // the single audio decoder. Cancel and fully close that socket
            // BEFORE starting the wake-word greeting on a new Live turn.
            if (!pending_wake_word_greeting_.empty()) {
                if (!IsAnnouncementActive() && GetDeviceState() == kDeviceStateIdle &&
                    (!protocol_ || !protocol_->IsAudioChannelOpened()) &&
                    audio_service_.IsPlaybackIdle() &&
                    RequestAnnouncement(pending_wake_word_greeting_kind_, pending_wake_word_greeting_.c_str())) {
                    ESP_LOGI(TAG, "[SPEECH] kind=%s phase=queued_after_interruption", pending_wake_word_greeting_kind_);
                    pending_wake_word_greeting_.clear();
                    pending_wake_word_since_us_ = 0;
                } else if (pending_wake_word_since_us_ &&
                           esp_timer_get_time() - pending_wake_word_since_us_ > 15000000) {
                    ESP_LOGW(TAG, "[SPEECH] kind=wake_word phase=greeting_retry_timeout");
                    pending_wake_word_greeting_.clear();
                    pending_wake_word_since_us_ = 0;
                    // Do not interrupt another user's conversation. The
                    // original wake-word path resumes when genuinely idle.
                }
            }
            // Voice-first wake follows the regular listening path only after
            // the announcement has ended AND its socket is fully closed.
            if (pending_wake_word_greeting_.empty() &&
                wake_word_handoff_pending_ && !IsAnnouncementActive()) {
                if (GetDeviceState() == kDeviceStateIdle &&
                    (!protocol_ || !protocol_->IsAudioChannelOpened()) &&
                    audio_service_.IsPlaybackIdle()) {
                    std::string word = std::move(wake_word_after_announcement_);
                    wake_word_after_announcement_.clear();
                    ESP_LOGI(TAG, "[SPEECH] kind=wake_word phase=handoff_to_listening");
                    BeginWakeWordInvoke(word);
                    wake_word_handoff_pending_ = false;
                } else if (GetDeviceState() == kDeviceStateListening ||
                           GetDeviceState() == kDeviceStateSpeaking) {
                    // User initiated another dialogue; do not replay the
                    // already-consumed wake word after that dialogue ends.
                    wake_word_after_announcement_.clear();
                    wake_word_handoff_pending_ = false;
                }
            }
            // A failed standalone request must not hold wake-up indefinitely.
            if (IsAnnouncementActive() && announcement_started_us_ &&
                esp_timer_get_time() - announcement_started_us_ > 75000000) {
                ESP_LOGW(TAG, "[SPEECH] kind=%s phase=timeout", AnnouncementTag(announcement_kind_.load()));
                FinishAnnouncement("timeout");
            }

            // Print debug info every 10 seconds
            if (clock_ticks_ % 10 == 0) {
                SystemInfo::PrintHeapStats();
                // SystemInfo::PrintTaskList();
                // SystemInfo::PrintTaskCpuUsage(pdMS_TO_TICKS(1000));
            }
        }
        if (bits & MAIN_EVENT_STARTUP_HINT_TIMEOUT) {
            HideStartupWakeWordHint();
        }
        ShowStartupWakeWordHint();
    }
}

void Application::TryStartLocalNetworkService() {
    if (!local_network_connected_ || !local_network_service_starter_ || IsAnnouncementActive()) return;
    const auto state = GetDeviceState();
    if (state != kDeviceStateStarting && state != kDeviceStateWifiConfiguring &&
        state != kDeviceStateActivating && state != kDeviceStateIdle) return;

    // Do not delay activation when local service resources are unavailable.
    // The service owns its retry interval; never call it from the Wi-Fi event task.
    if (local_network_service_starter_()) local_network_service_starter_ = nullptr;
}

void Application::HandleNetworkConnectedEvent() {
    ESP_LOGI(TAG, "Network connected");
    local_network_connected_ = true;
    TryStartLocalNetworkService();
    auto state = GetDeviceState();

    if (state == kDeviceStateStarting || state == kDeviceStateWifiConfiguring) {
        // Network is ready, start activation
        SetDeviceState(kDeviceStateActivating);
        if (activation_task_handle_ != nullptr) {
            ESP_LOGW(TAG, "Activation task already running");
            return;
        }

        xTaskCreate(
            [](void* arg) {
                Application* app = static_cast<Application*>(arg);
                app->ActivationTask();
                app->activation_task_handle_ = nullptr;
                vTaskDelete(NULL);
            },
            "activation", 4096 * 2, this, 2, &activation_task_handle_);
    }

    // Update the status bar immediately to show the network state
    auto display = Board::GetInstance().GetDisplay();
    display->UpdateStatusBar(true);
}

void Application::HandleNetworkDisconnectedEvent() {
    local_network_connected_ = false;
    // Close current conversation when network disconnected
    auto state = GetDeviceState();
    if (state == kDeviceStateNotifying) {
        StopNotification();
    }
    if (state == kDeviceStateConnecting || state == kDeviceStateListening ||
        state == kDeviceStateSpeaking) {
        ESP_LOGI(TAG, "Closing audio channel due to network disconnection");
        protocol_->CloseAudioChannel();
    }

    // Update the status bar immediately to show the network state
    auto display = Board::GetInstance().GetDisplay();
    display->UpdateStatusBar(true);
}

void Application::MaybeReconnectAudioChannel() {
    if (low_battery_mode_.load()) return;
    if (!auto_reconnect_pending_ || !is_websocket_protocol_ || !protocol_ ||
        wake_word_handoff_pending_ || IsAnnouncementActive() ||
        GetDeviceState() != kDeviceStateIdle || esp_timer_get_time() < auto_reconnect_due_us_) {
        return;
    }

    // Only unexpected losses while a conversation was active arm this path.
    // Manual stop, wake-word invocation, and normal connection attempts retain
    // their existing behavior. Backoff retries do not play error sounds.
    if (!SetDeviceState(kDeviceStateConnecting)) {
        return;
    }
    Board::GetInstance().SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
    ++auto_reconnect_attempts_;
    ESP_LOGI(TAG, "Automatically reconnecting WebSocket (attempt %u)", auto_reconnect_attempts_);

    auto* ws = static_cast<WebsocketProtocol*>(protocol_.get());
    if (ws->ReconnectAudioChannel()) {
        auto_reconnect_pending_ = false;
        auto_reconnect_attempts_ = 0;
        ESP_LOGI(TAG, "WebSocket reconnected; resuming microphone listening");
        SetListeningMode(listening_mode_);
        return;
    }

    Board::GetInstance().SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
    SetDeviceState(kDeviceStateIdle);
    const unsigned shift = std::min(auto_reconnect_attempts_, 4u);
    const unsigned delay_seconds = std::min(30u, 2u << shift);
    auto_reconnect_due_us_ = esp_timer_get_time() + static_cast<int64_t>(delay_seconds) * 1000000;
    ESP_LOGW(TAG, "WebSocket reconnect failed; next attempt in %u seconds", delay_seconds);
}

void Application::HandleActivationDoneEvent() {
    ESP_LOGI(TAG, "Activation done");

    LogMemorySnapshot("activation_done");
    SystemInfo::PrintHeapStats();
    SetDeviceState(kDeviceStateIdle);

    has_server_time_ = ota_ && ota_->HasServerTime();

    // Protocol start may have already raised MAIN_EVENT_ERROR. Do not replace
    // that alert with the "ready" UI/sound — the main loop can process both
    // events back-to-back because the activation task is lower priority.
    const bool has_error = HasLastErrorMessage();
    if (!startup_completed_) {
        startup_completed_ = true;
        pending_startup_wake_word_hint_ = !has_error;
    }
    if (!has_error) {
        auto display = Board::GetInstance().GetDisplay();
        const std::string version = ota_ ? ota_->GetCurrentVersion() : esp_app_get_description()->version;
        std::string message = std::string(Lang::Strings::VERSION) + version;
        display->ShowNotification(message.c_str());
        display->SetChatMessage("system", "");
    }

    // Release OTA object after activation is complete
    ota_.reset();
    LogMemorySnapshot("after_ota_release");
    auto& board = Board::GetInstance();
    board.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);

    if (!has_error) {
        Schedule([this]() {
            // Play the success sound to indicate the device is ready
            audio_service_.PlaySound(Lang::Sounds::OGG_SUCCESS);
        });
    }
}

void Application::ActivationTask() {
    // Apply installed assets independently of provisioning.
    CheckAssetsVersion();

    bool direct = false;
    bool use_mqtt = false;
#if CONFIG_BOARD_TYPE_M5STACK_STACK_CHAN
    using stackchan::config::server_boot::Protocol;
    const auto stored = stackchan::config::server_boot::ReadDirectProtocol();
    if (stored != Protocol::Legacy) {
        direct = true;
        use_mqtt = stored == Protocol::Mqtt;
        ESP_LOGI(TAG, "Starting directly from stored %s configuration", use_mqtt ? "MQTT" : "WebSocket");
        // OTA no longer supplies the clock. Existing SNTP/RTC sync remains.
        GetHAL().startSntp();
    }
#endif
    if (!direct) {
        // Transitional fallback until independent Wi-Fi/server setup is ready.
        ota_ = std::make_unique<Ota>();
        CheckNewVersion();
        use_mqtt = ota_->HasMqttConfig() || !ota_->HasWebsocketConfig();
        if (!ota_->HasMqttConfig() && !ota_->HasWebsocketConfig()) {
            ESP_LOGW(TAG, "No protocol specified in the OTA config, using MQTT");
        }
    }

    InitializeProtocol(use_mqtt);
    xEventGroupSetBits(event_group_, MAIN_EVENT_ACTIVATION_DONE);
}

void Application::CheckAssetsVersion() {
    // Only allow CheckAssetsVersion to be called once
    if (assets_version_checked_) {
        return;
    }
    assets_version_checked_ = true;

    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto& assets = Assets::GetInstance();

    if (!assets.partition_valid()) {
        ESP_LOGW(TAG, "Assets partition is disabled for board %s", BOARD_NAME);
        return;
    }

    // Self-hosted operation: do not consume a pending download_url on boot.
    // Assets::Apply below is still required for the installed UI/voice assets.
    // Apply existing assets without checking for or downloading updates.
    assets.Apply();
    display->SetChatMessage("system", "");
    // "robot_2" is declared only by a board shipping its own pack: esp-vocat maps it to
    // neutral.eaf, and the rgb-matrix display special-cases the string. None of the shared
    // esp_emote_assets packs (320_240, 360_360, 1024_600) declares it, so on every board that
    // uses one this call fails, logs two errors and leaves whatever face was already up.
    // Fall back to an emotion all three packs do declare.
    if (!display->SetEmotion("robot_2")) {
        display->SetEmotion("neutral");
    }
}

void Application::CheckNewVersion() {
    const int MAX_RETRY = 10;
    int retry_count = 0;
    int challenge_rounds = 0;
    int retry_delay = 10;  // Initial retry delay in seconds

    auto& board = Board::GetInstance();
    while (true) {
        auto display = board.GetDisplay();
        display->SetStatus(Lang::Strings::ACTIVATION);

        auto check = ota_->CheckVersion();
        if (!check) {
            retry_count++;
            if (retry_count >= MAX_RETRY) {
                ESP_LOGE(TAG, "Too many retries, exit version check");
                return;
            }

            const auto& err = check.error();
            char error_message[160];
            int error_message_length =
                snprintf(error_message, sizeof(error_message), "%s", err.ToString().c_str());
            if (error_message_length < 0 ||
                error_message_length >= static_cast<int>(sizeof(error_message))) {
                snprintf(error_message, sizeof(error_message), "%s", err.Message());
            }

            char buffer[320];
            int alert_message_length =
                snprintf(buffer, sizeof(buffer), Lang::Strings::CHECK_NEW_VERSION_FAILED,
                         retry_delay, error_message);
            if (alert_message_length < 0 ||
                alert_message_length >= static_cast<int>(sizeof(buffer))) {
                snprintf(buffer, sizeof(buffer), "%s", err.Message());
            }
            Alert(Lang::Strings::ERROR, buffer, "cloud_off", Lang::Sounds::OGG_EXCLAMATION);

            ESP_LOGW(TAG, "Check new version failed, retry in %d seconds (%d/%d)", retry_delay,
                     retry_count, MAX_RETRY);
            for (int i = 0; i < retry_delay; i++) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                if (GetDeviceState() == kDeviceStateIdle) {
                    break;
                }
            }
            retry_delay *= 2;  // Double the retry delay
            continue;
        }
        retry_count = 0;
        retry_delay = 10;  // Reset retry delay

        // The endpoint response also provisions the WebSocket target.
        // Never auto-install firmware even if a version/force flag is returned.
        // Preserve partition confirmation to avoid bootloader rollback.
        // Mark the current version as valid
        ota_->MarkCurrentVersionValid();
        if (ota_->RequiresAppBinding()) {
            GetHAL().requestAppSetup();
        }
        if (!ota_->HasActivationChallenge()) {
            // Exit the loop if done checking new version
            break;
        }

        display->SetStatus(Lang::Strings::ACTIVATION);
        // Keep automatic device authentication bounded; manual binding uses the App.
        if (++challenge_rounds > 3) {
            GetHAL().requestAppSetup();
        }

        // This will block the loop until the activation is done or timeout
        for (int i = 0; i < 10; ++i) {
            ESP_LOGI(TAG, "Activating... %d/%d", i + 1, 10);
            esp_err_t err = ota_->Activate();
            if (err == ESP_OK) {
                break;
            } else if (err == ESP_ERR_TIMEOUT) {
                vTaskDelay(pdMS_TO_TICKS(3000));
            } else {
                vTaskDelay(pdMS_TO_TICKS(10000));
            }
            if (GetDeviceState() == kDeviceStateIdle) {
                break;
            }
        }
    }
}

void Application::InitializeProtocol(bool use_mqtt) {
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto codec = board.GetAudioCodec();

    display->SetStatus(Lang::Strings::LOADING_PROTOCOL);

    is_websocket_protocol_ = false;
    auto_reconnect_pending_ = false;
    if (use_mqtt) {
        protocol_ = std::make_unique<MqttProtocol>();
    } else {
        protocol_ = std::make_unique<WebsocketProtocol>();
        is_websocket_protocol_ = true;
    }

    if (is_websocket_protocol_) {
        protocol_->OnDisconnected([this]() {
            const DeviceState lost_state = GetDeviceState();
            const unsigned generation = protocol_ ? protocol_->callback_generation() : 0;
            Schedule([this, lost_state, generation]() {
                if (!is_websocket_protocol_ || !protocol_ ||
                    protocol_->connection_generation() != generation ||
                    (lost_state != kDeviceStateListening && lost_state != kDeviceStateSpeaking)) {
                    return;
                }
                auto_reconnect_pending_ = true;
                auto_reconnect_attempts_ = 0;
                auto_reconnect_due_us_ = esp_timer_get_time() + 2000000;
                ESP_LOGW(TAG, "Unexpected WebSocket loss while chatting; reconnect scheduled in 2 seconds");
            });
        });
    }
    protocol_->OnConnected([this]() { Schedule([this]() { DismissAlert(); }); });

    protocol_->OnNetworkError([this](const std::string& message) {
        SetLastErrorMessage(message);
        xEventGroupSetBits(event_group_, MAIN_EVENT_ERROR);
    });

    protocol_->OnIncomingAudio([this](std::unique_ptr<AudioStreamPacket> packet) {
        HandleIncomingTtsAudio(std::move(packet));
    });

    protocol_->OnAudioChannelOpened([this, codec]() {
        const unsigned generation = protocol_ ? protocol_->callback_generation() : 0;
        if (is_websocket_protocol_) remote_mcp_generation_.store(generation);
        Schedule([this, codec, generation]() {
            if (!protocol_ || protocol_->connection_generation() != generation) return;
            auto& board = Board::GetInstance();
            board.SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
            // WebSocket uses this callback after server hello; unlike MQTT it
            // does not invoke OnConnected(). Clear a previous connection error
            // without resetting the connecting/listening UI or other subtitles.
            if (HasLastErrorMessage()) {
                DismissAlert();
                board.GetDisplay()->SetChatMessage("system", "");
            }
            if (protocol_->server_sample_rate() != codec->output_sample_rate()) {
                ESP_LOGW(TAG,
                         "Server sample rate %d does not match device output sample rate %d, "
                         "resampling may cause distortion",
                         protocol_->server_sample_rate(), codec->output_sample_rate());
            }
        });
    });

    protocol_->OnAudioChannelClosed([this]() {
        const unsigned generation = protocol_ ? protocol_->callback_generation() : 0;
        if (is_websocket_protocol_) {
            unsigned expected = generation;
            remote_mcp_generation_.compare_exchange_strong(expected, 0);
        }
        Schedule([this, generation]() {
            if (protocol_ && protocol_->connection_generation() != generation) {
                ESP_LOGI(TAG, "Ignoring close callback from websocket generation %u", generation);
                return;
            }
            Board::GetInstance().SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
            ClearPendingTtsAudio();
            // Preserve ordering with queued TTS stop/playback events. Clearing
            // the flag on the network callback thread could re-enable listening.
            const AnnouncementKind interrupted = announcement_kind_.exchange(AnnouncementKind::None);
            if (interrupted != AnnouncementKind::None) {
                ESP_LOGW(TAG, "[SPEECH] kind=%s phase=transport_closed", AnnouncementTag(interrupted));
                announcement_started_us_ = 0;
                tts_stop_pending_ = false;
                announcement_subtitles_linger_ = true;
            }
            auto display = Board::GetInstance().GetDisplay();
            if (!announcement_subtitles_linger_) display->SetChatMessage("system", "");
            SetDeviceState(kDeviceStateIdle);
            // A prior socket close may be delivered after the queued UI
            // events. Resume the wake-word only from the clock tick, after
            // all old transport callbacks have settled.
        });
    });

    protocol_->OnIncomingJson([this, display](const cJSON* root) {
        // Parse JSON data
        auto type = cJSON_GetObjectItem(root, "type");
        if (!cJSON_IsString(type)) {
            ESP_LOGW(TAG, "Incoming JSON message has no type");
            return;
        }
        if (strcmp(type->valuestring, "notify") == 0) {
            auto audio_url = cJSON_GetObjectItem(root, "audio_url");
            if (!cJSON_IsString(audio_url) || audio_url->valuestring[0] == '\0') {
                ESP_LOGW(TAG, "Notify message requires audio_url");
                return;
            }

            std::vector<NotifySubtitle> subtitles;
            auto subtitles_json = cJSON_GetObjectItem(root, "subtitles");
            if (subtitles_json != nullptr && !cJSON_IsArray(subtitles_json)) {
                ESP_LOGW(TAG, "Notify subtitles must be an array");
                return;
            }
            if (cJSON_IsArray(subtitles_json)) {
                cJSON* item = nullptr;
                cJSON_ArrayForEach (item, subtitles_json) {
                    auto start_ms = cJSON_GetObjectItem(item, "start_ms");
                    auto text = cJSON_GetObjectItem(item, "text");
                    if (!cJSON_IsNumber(start_ms) || start_ms->valuedouble < 0 ||
                        start_ms->valuedouble > std::numeric_limits<uint32_t>::max() ||
                        !cJSON_IsString(text)) {
                        ESP_LOGW(TAG, "Ignoring invalid notify subtitle");
                        continue;
                    }
                    subtitles.push_back({.start_ms = static_cast<uint32_t>(start_ms->valuedouble),
                                         .text = text->valuestring});
                }
            }

            Schedule([this, url = std::string(audio_url->valuestring),
                      subtitles = std::move(subtitles)]() mutable {
                StartNotification(std::move(url), std::move(subtitles));
            });
        } else if (strcmp(type->valuestring, "glyphs") == 0) {
            // Standalone speech gets bounded glyph batches from Docker before
            // each subtitle. The normal TTS/STT glyph path is unchanged.
            if (!IsAnnouncementActive()) return;
            std::vector<TextGlyph> glyphs;
            uint8_t bpp = 0;
            if (TextGlyphPayload::Parse(root, glyphs, bpp) && !glyphs.empty()) {
                Schedule([display, glyphs = std::move(glyphs), bpp]() {
                    display->AddTextGlyphs(glyphs, bpp);
                });
            }
        } else if (strcmp(type->valuestring, "speak") == 0) {
            auto state = cJSON_GetObjectItem(root, "state");
            auto kind = cJSON_GetObjectItem(root, "kind");
            if (cJSON_IsString(state)) {
                const char* phase = strcmp(state->valuestring, "completed") == 0 ?
                    "server_stream_completed" : state->valuestring;
                ESP_LOGI(TAG, "[SPEECH] kind=%s phase=%s session=%s",
                         cJSON_IsString(kind) ? kind->valuestring :
                           AnnouncementTag(announcement_kind_.load()),
                         phase, protocol_ ? protocol_->session_id().c_str() : "");
                if (strcmp(state->valuestring, "error") == 0)
                    Schedule([this]() { FinishAnnouncement("server_error"); });
            }
        } else if (strcmp(type->valuestring, "tts") == 0) {
            auto state = cJSON_GetObjectItem(root, "state");
            if (!cJSON_IsString(state)) {
                return;
            }
            auto speech_kind = cJSON_GetObjectItem(root, "kind");
            const char* kind_tag = cJSON_IsString(speech_kind) ?
                                   speech_kind->valuestring : "conversation";
            if (strcmp(state->valuestring, "start") == 0 ||
                strcmp(state->valuestring, "stop") == 0) {
                ESP_LOGI(TAG, "[SPEECH] kind=%s phase=tts_%s transport=ws12800",
                         kind_tag, state->valuestring);
            }
            if (strcmp(state->valuestring, "start") == 0) {
                PrepareTtsStart();
                if (!Schedule([this]() {
                    aborted_ = false;
                    tts_stop_pending_ = false;
                    last_speech_subtitle_tick_ms_.store(0);
                    ActivateTtsPlayback();
                })) {
                    ClearPendingTtsAudio();
                }
            } else if (strcmp(state->valuestring, "stop") == 0) {
                Schedule([this]() {
                    if (GetDeviceState() != kDeviceStateSpeaking) return;
                    if (!audio_service_.IsPlaybackIdle()) {
                        tts_stop_pending_ = true;
                        ESP_LOGI(TAG, "TTS stop received; awaiting PCM playback drain");
                        return;
                    }
                    tts_stop_pending_ = false;
                    if (IsAnnouncementActive()) FinishAnnouncement();
                    else FinishConversationPlayback();
                });
            } else if (strcmp(state->valuestring, "sentence_start") == 0) {
                auto text = cJSON_GetObjectItem(root, "text");
                if (cJSON_IsString(text)) {
                    std::vector<TextGlyph> glyphs;
                    uint8_t bpp = 0;
                    if (!TextGlyphPayload::Parse(root, glyphs, bpp)) {
                        glyphs.clear();
                    }
                    ESP_LOGI(TAG, "<< %s", text->valuestring);
                    Schedule([display, message = std::string(text->valuestring),
                              glyphs = std::move(glyphs), bpp]() {
                        display->AddTextGlyphs(glyphs, bpp);
                        display->SetChatMessage("assistant", message.c_str());
                    });
                }
            }
        } else if (strcmp(type->valuestring, "stt") == 0) {
            auto text = cJSON_GetObjectItem(root, "text");
            if (cJSON_IsString(text)) {
                std::vector<TextGlyph> glyphs;
                uint8_t bpp = 0;
                if (!TextGlyphPayload::Parse(root, glyphs, bpp)) {
                    glyphs.clear();
                }
                ESP_LOGI(TAG, ">> %s", text->valuestring);
                Schedule([display, message = std::string(text->valuestring),
                          glyphs = std::move(glyphs), bpp]() {
                    display->AddTextGlyphs(glyphs, bpp);
                    display->SetChatMessage("user", message.c_str());
                });
            }
        } else if (strcmp(type->valuestring, "llm") == 0) {
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (cJSON_IsString(emotion)) {
                Schedule([display, emotion_str = std::string(emotion->valuestring)]() {
                    display->SetEmotion(emotion_str.c_str());
                });
            }
        } else if (strcmp(type->valuestring, "mcp") == 0) {
            auto payload = cJSON_GetObjectItem(root, "payload");
            if (cJSON_IsObject(payload)) {
                if (is_websocket_protocol_ && protocol_) {
                    const unsigned generation = protocol_->callback_generation();
                    // Inventory replies can arrive during the hello handoff;
                    // send on the main task only if the same socket is open.
                    auto current = [this, generation]() {
                        return generation != 0 && remote_mcp_generation_.load() == generation;
                    };
                    McpServer::GetInstance().ParseMessage(payload,
                        [this, generation](const std::string& response) {
                            Schedule([this, generation, response]() {
                                if (!protocol_ || protocol_->connection_generation() != generation ||
                                    !protocol_->IsAudioChannelOpened()) return;
                                protocol_->SendMcpMessage(response);
                                if (mcp_broadcast_callback_) mcp_broadcast_callback_(response);
                            });
                        }, current);
                } else {
                    McpServer::GetInstance().ParseMessage(payload);
                }
            }
        } else if (strcmp(type->valuestring, "system") == 0) {
            auto command = cJSON_GetObjectItem(root, "command");
            if (cJSON_IsString(command)) {
                ESP_LOGI(TAG, "System command: %s", command->valuestring);
                if (strcmp(command->valuestring, "reboot") == 0) {
                    // Do a reboot if user requests a OTA update
                    Schedule([this]() { Reboot(); });
                } else {
                    ESP_LOGW(TAG, "Unknown system command: %s", command->valuestring);
                }
            }
        } else if (strcmp(type->valuestring, "alert") == 0) {
            auto status = cJSON_GetObjectItem(root, "status");
            auto message = cJSON_GetObjectItem(root, "message");
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (cJSON_IsString(status) && cJSON_IsString(message) && cJSON_IsString(emotion)) {
                Alert(status->valuestring, message->valuestring, emotion->valuestring,
                      Lang::Sounds::OGG_VIBRATION);
            } else {
                ESP_LOGW(TAG, "Alert command requires status, message and emotion");
            }
#if CONFIG_RECEIVE_CUSTOM_MESSAGE
        } else if (strcmp(type->valuestring, "custom") == 0) {
            auto payload = cJSON_GetObjectItem(root, "payload");
            CJsonStringUniquePtr root_json(cJSON_PrintUnformatted(root));
            ESP_LOGI(TAG, "Received custom message: %s", root_json ? root_json.get() : "");
            if (cJSON_IsObject(payload)) {
                CJsonStringUniquePtr payload_json(cJSON_PrintUnformatted(payload));
                if (payload_json) {
                    Schedule(
                        [this, display, payload_str = std::string(payload_json.get())]() {
                            display->SetChatMessage("system", payload_str.c_str());
                        });
                }
            } else {
                ESP_LOGW(TAG, "Invalid custom message format: missing payload");
            }
#endif
        } else {
            ESP_LOGW(TAG, "Unknown message type: %s", type->valuestring);
        }
    });

    protocol_->Start();
}

void Application::Alert(const char* status, const char* message, const char* emotion,
                        const std::string_view& sound) {
    ESP_LOGW(TAG, "Alert [%s] %s: %s", emotion, status, message);
    auto display = Board::GetInstance().GetDisplay();
    display->SetStatus(status);
    display->SetEmotion(emotion);
    display->SetChatMessage("system", message);
    if (!sound.empty()) {
        audio_service_.PlaySound(sound);
    }
}

void Application::DismissAlert() {
    ClearLastErrorMessage();
    if (GetDeviceState() == kDeviceStateIdle) {
        auto display = Board::GetInstance().GetDisplay();
        display->SetStatus(Lang::Strings::STANDBY);
        display->SetEmotion("neutral");
        display->SetChatMessage("system", "");
    }
}

void Application::ToggleChatState() { xEventGroupSetBits(event_group_, MAIN_EVENT_TOGGLE_CHAT); }

void Application::StartListening() { xEventGroupSetBits(event_group_, MAIN_EVENT_START_LISTENING); }

void Application::StopListening() { xEventGroupSetBits(event_group_, MAIN_EVENT_STOP_LISTENING); }

void Application::HandleToggleChatEvent() {
    if (low_battery_mode_.load()) return;
    auto state = GetDeviceState();

    if (state == kDeviceStateNotifying) {
        StopNotification();
        state = kDeviceStateIdle;
    }

    if (state == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
        return;
    } else if (state == kDeviceStateWifiConfiguring) {
        audio_service_.EnableAudioTesting(true);
        SetDeviceState(kDeviceStateAudioTesting);
        return;
    } else if (state == kDeviceStateAudioTesting) {
        audio_service_.EnableAudioTesting(false);
        SetDeviceState(kDeviceStateWifiConfiguring);
        return;
    }

    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        return;
    }

    if (state == kDeviceStateIdle) {
        ListeningMode mode = GetDefaultListeningMode();
        if (!protocol_->IsAudioChannelOpened()) {
            SetDeviceState(kDeviceStateConnecting);
            // Schedule to let the state change be processed first (UI update)
            Schedule([this, mode]() { ContinueOpenAudioChannel(mode); });
            return;
        }
        SetListeningMode(mode);
    } else if (state == kDeviceStateSpeaking) {
        AbortSpeaking(kAbortReasonNone);
    } else if (state == kDeviceStateListening) {
        protocol_->CloseAudioChannel();
    }
}

void Application::ContinueOpenAudioChannel(ListeningMode mode) {
    if (low_battery_mode_.load()) return;
    // Check state again in case it was changed during scheduling
    if (GetDeviceState() != kDeviceStateConnecting) {
        return;
    }

    // Switch to performance mode before connecting to reduce latency
    auto& board = Board::GetInstance();
    board.SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);

    if (!protocol_->IsAudioChannelOpened()) {
        if (!protocol_->OpenAudioChannel()) {
            // Return to idle so the device is not stuck in the connecting
            // state (not every failure path reports a network error)
            board.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
            SetDeviceState(kDeviceStateIdle);
            return;
        }
    }

    auto_reconnect_pending_ = false;
    SetListeningMode(mode);
}

void Application::HandleStartListeningEvent() {
    if (low_battery_mode_.load()) return;
    auto state = GetDeviceState();

    if (state == kDeviceStateNotifying) {
        StopNotification();
        state = kDeviceStateIdle;
    }

    if (state == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
        return;
    } else if (state == kDeviceStateWifiConfiguring) {
        audio_service_.EnableAudioTesting(true);
        SetDeviceState(kDeviceStateAudioTesting);
        return;
    }

    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        return;
    }

    if (state == kDeviceStateIdle) {
        if (!protocol_->IsAudioChannelOpened()) {
            SetDeviceState(kDeviceStateConnecting);
            // Schedule to let the state change be processed first (UI update)
            Schedule([this]() { ContinueOpenAudioChannel(kListeningModeManualStop); });
            return;
        }
        SetListeningMode(kListeningModeManualStop);
    } else if (state == kDeviceStateSpeaking) {
        AbortSpeaking(kAbortReasonNone);
        SetListeningMode(kListeningModeManualStop);
    }
}

void Application::HandleStopListeningEvent() {
    auto state = GetDeviceState();

    if (state == kDeviceStateNotifying) {
        StopNotification();
    } else if (state == kDeviceStateAudioTesting) {
        audio_service_.EnableAudioTesting(false);
        SetDeviceState(kDeviceStateWifiConfiguring);
        return;
    } else if (state == kDeviceStateListening) {
        if (protocol_) {
            protocol_->SendStopListening();
        }
        SetDeviceState(kDeviceStateIdle);
    }
}

void Application::HandleWakeWordDetectedEvent() {
    if (low_battery_mode_.load()) return;
    if (!protocol_) {
        return;
    }

    auto state = GetDeviceState();
    auto wake_word = audio_service_.GetLastWakeWord();
    ESP_LOGI(TAG, "Wake word detected: %s (state: %d)", wake_word.c_str(), (int)state);
    // Board-specific perception runs independently and must not delay the
    // normal wake-word state transition or audio-channel connection.
    Board::GetInstance().OnWakeWordDetected();

    const auto active_kind = announcement_kind_.load();
    const bool interrupting_monologue = active_kind == AnnouncementKind::Monologue ||
        active_kind == AnnouncementKind::ConversationRemark;
    if (interrupting_monologue || active_kind == AnnouncementKind::DrowsyReply) {
        pending_wake_word_greeting_kind_ = "interrupt_reply";
        pending_wake_word_greeting_ = interrupting_monologue && state == kDeviceStateSpeaking
            ? (esp_random() % 8 == 0 ? "あ、いや、なんでもない。" : "あ、ごめん。なになに？") : "";
        pending_wake_word_since_us_ = pending_wake_word_greeting_.empty() ? 0 : esp_timer_get_time();
        wake_word_after_announcement_ = wake_word;
        wake_word_handoff_pending_ = true;
        if (protocol_->IsAudioChannelOpened()) protocol_->SendAbortSpeaking(kAbortReasonWakeWordDetected);
        audio_service_.ResetDecoder();
        FinishAnnouncement("canceled_wake_word");
        return;
    }
    const bool sleep_sound_connection =
        (state == kDeviceStateConnecting || state == kDeviceStateSpeaking) &&
        (active_kind == AnnouncementKind::Snore || active_kind == AnnouncementKind::SleepTalk);
    if (sleep_sound_connection) {
        // The wake word takes precedence even if its configured greeting is
        // blank or temporarily unavailable: it must still reach listening.
        // Never apply this special interruption to normal conversations.
        const std::string greeting = wake_word_announcement_provider_
            ? wake_word_announcement_provider_() : std::string{};
        pending_wake_word_greeting_ = greeting;
        pending_wake_word_greeting_kind_ = "wake_word";
        pending_wake_word_since_us_ = greeting.empty() ? 0 : esp_timer_get_time();
        wake_word_after_announcement_ = wake_word;
        wake_word_handoff_pending_ = true;
        ESP_LOGI(TAG, "[SPEECH] kind=wake_word phase=interrupt_sleep_sound greeting=%s",
                 greeting.empty() ? "skipped" : "pending");
        CancelSleepAnnouncement();
        return;
    }

    if (state == kDeviceStateIdle) {
        // Sleep-only S3 greeting: do not begin microphone capture until its
        // dedicated text-only Live turn and PCM playback have completed.
        if (wake_word_announcement_provider_) {
            const std::string greeting = wake_word_announcement_provider_();
            if (!greeting.empty() && RequestAnnouncement("wake_word", greeting.c_str())) {
                wake_word_after_announcement_ = wake_word;
                wake_word_handoff_pending_ = true;
                ESP_LOGI(TAG, "[SPEECH] kind=wake_word phase=queued_before_listening");
                return;
            }
        }
        BeginWakeWordInvoke(wake_word);
    } else if (state == kDeviceStateNotifying) {
        StopNotification();
        BeginWakeWordInvoke(wake_word);
    } else if (state == kDeviceStateSpeaking || state == kDeviceStateListening) {
        AbortSpeaking(kAbortReasonWakeWordDetected);
        // Clear send queue to avoid sending residues to server
        while (audio_service_.PopPacketFromSendQueue())
            ;

        if (state == kDeviceStateListening) {
            protocol_->SendStartListening(GetDefaultListeningMode());
            audio_service_.ResetDecoder();
            audio_service_.PlaySound(Lang::Sounds::OGG_POPUP);
            // Re-enable wake word detection as it was stopped by the detection itself
            audio_service_.EnableWakeWordDetection(true);
        } else {
            // Play popup sound and start listening again
            play_popup_on_listening_ = true;
            SetListeningMode(GetDefaultListeningMode());
        }
    } else if (state == kDeviceStateActivating) {
        // Restart the activation check if the wake word is detected during activation
        SetDeviceState(kDeviceStateIdle);
    }
}

void Application::BeginWakeWordInvoke(const std::string& wake_word) {
    if (low_battery_mode_.load()) return;
    // Must run in the main task with the device in idle state.
    // The encoder's PSRAM task stack (24 KiB for WakeNet) is only needed
    // when the captured wake-word audio is actually sent to the server.
#if CONFIG_SEND_WAKE_WORD_DATA
    audio_service_.EncodeWakeWord();
#endif

    // Always pass through the connecting state, even if the audio channel is
    // already opened. ContinueWakeWordInvoke() rejects any other state, so
    // skipping this transition would silently drop the wake word invocation.
    if (!SetDeviceState(kDeviceStateConnecting)) {
        // Wake word detection was stopped by the detection itself; restore it
        // so the device does not become unresponsive to wake words.
        audio_service_.EnableWakeWordDetection(true);
        return;
    }

    if (!protocol_->IsAudioChannelOpened()) {
        // Schedule to let the state change be processed first (UI update),
        // then continue with OpenAudioChannel which may block for ~1 second
        Schedule([this, wake_word]() { ContinueWakeWordInvoke(wake_word); });
        return;
    }
    // Channel already opened, continue directly
    ContinueWakeWordInvoke(wake_word);
}

void Application::ContinueWakeWordInvoke(const std::string& wake_word) {
    if (low_battery_mode_.load()) return;
    // Check state again in case it was changed during scheduling
    if (GetDeviceState() != kDeviceStateConnecting) {
        return;
    }

    // Switch to performance mode before connecting to reduce latency
    auto& board = Board::GetInstance();
    board.SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);

    if (!protocol_->IsAudioChannelOpened()) {
        if (!protocol_->OpenAudioChannel()) {
            // Return to idle so the device is not stuck in the connecting
            // state (not every failure path reports a network error), and
            // wake word detection is re-enabled by the idle state handler.
            board.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
            SetDeviceState(kDeviceStateIdle);
            return;
        }
    }

    auto_reconnect_pending_ = false;
    ESP_LOGI(TAG, "Wake word detected: %s", wake_word.c_str());
#if CONFIG_SEND_WAKE_WORD_DATA
    // Encode and send the wake word data to the server
    while (auto packet = audio_service_.PopWakeWordPacket()) {
        protocol_->SendAudio(std::move(packet));
    }
    // Set the chat state to wake word detected
    protocol_->SendWakeWordDetected(wake_word);
    SetListeningMode(GetDefaultListeningMode());
#else
    // Set flag to play popup sound after state changes to listening
    // (PlaySound here would be cleared by ResetDecoder in EnableVoiceProcessing)
    play_popup_on_listening_ = true;
    SetListeningMode(GetDefaultListeningMode());
#endif
}

void Application::HandleStateChangedEvent() {
    DeviceState new_state = state_machine_.GetState();
    clock_ticks_ = 0;
    // Any state change invalidates a pending deferred listening start;
    // the Listening case below re-arms it when needed.
    pending_listening_start_ = false;

    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto led = board.GetLed();
    led->OnStateChanged();

    switch (new_state) {
        case kDeviceStateUnknown:
        case kDeviceStateIdle:
            quiet_end_pending_ = false;
            // Keep a just-raised network error visible. SetDeviceState(idle)
            // queues STATE_CHANGED after Alert(), and the idle handler would
            // otherwise wipe the status, emotion, and chat message.
            if (!HasLastErrorMessage()) {
                display->SetStatus(Lang::Strings::STANDBY);
                if (!announcement_subtitles_linger_)
                    display->ClearChatMessages();  // Keep standalone review
                display->SetEmotion(
                    "neutral");  // Then set emotion (wechat mode checks child count)
            }
            audio_service_.EnableVoiceProcessing(false);
            audio_service_.EnableWakeWordDetection(!low_battery_mode_.load());
            break;
        case kDeviceStateConnecting:
            announcement_subtitles_linger_ = false;
            display->SetStatus(Lang::Strings::CONNECTING);
            display->SetEmotion("neutral");
            display->SetChatMessage("system", "");
            break;
        case kDeviceStateListening:
            display->SetStatus(Lang::Strings::LISTENING);
            display->SetEmotion("neutral");

            // Make sure the audio processor is running
            if (play_popup_on_listening_ || !audio_service_.IsAudioProcessorRunning()) {
                // For auto mode, wait for the playback queue to drain before enabling
                // voice processing. This prevents audio truncation when STOP arrives
                // late due to network jitter. Instead of blocking the main loop here,
                // defer the start until MAIN_EVENT_PLAYBACK_DRAINED arrives.
                if (listening_mode_ == kListeningModeAutoStop && !audio_service_.IsPlaybackIdle()) {
                    pending_listening_start_ = true;
                } else {
                    StartListeningAudio();
                }
            } else {
                ConfigureWakeWordForListening();
            }
            break;
        case kDeviceStateSpeaking:
            display->SetStatus(Lang::Strings::SPEAKING);

            if (listening_mode_ != kListeningModeRealtime) {
                audio_service_.EnableVoiceProcessing(false);
                // Only AFE wake word can be detected in speaking mode
                audio_service_.EnableWakeWordDetection(!low_battery_mode_.load() && audio_service_.IsAfeWakeWord());
            }
            break;
        case kDeviceStateNotifying:
            display->SetStatus(Lang::Strings::SPEAKING);
            audio_service_.EnableVoiceProcessing(false);
            audio_service_.EnableWakeWordDetection(!low_battery_mode_.load() && audio_service_.IsAfeWakeWord());
            break;
        case kDeviceStateWifiConfiguring:
            audio_service_.EnableVoiceProcessing(false);
            audio_service_.EnableWakeWordDetection(false);
            break;
        default:
            // Do nothing
            break;
    }
}

void Application::ShowStartupWakeWordHint() {
    // A coalesced activation/state event can queue another idle UI cleanup.
    // Wait for it before adding the hint, or ClearChatMessages() would erase it.
    if (!pending_startup_wake_word_hint_ ||
        (xEventGroupGetBits(event_group_) & MAIN_EVENT_STATE_CHANGED)) {
        return;
    }
    auto display = Board::GetInstance().GetDisplay();
    const bool dedicated_startup = display->SupportsStartupSpeech();
    // The activation chime uses the same decoder as a spoken announcement.
    // Do not reset or preempt it. Wait briefly for its normal drain signal.
    if (dedicated_startup && is_websocket_protocol_ && !audio_service_.IsPlaybackIdle()) {
        const int64_t now_us = esp_timer_get_time();
        if (startup_hint_voice_wait_since_us_ == 0)
            startup_hint_voice_wait_since_us_ = now_us;
        if (now_us - startup_hint_voice_wait_since_us_ < 5000000) return;
        ESP_LOGW(TAG, "[STARTUP] audio still busy; showing original hint instead");
    }
    pending_startup_wake_word_hint_ = false;
    const auto state = GetDeviceState();
    const bool wake_word_running = audio_service_.IsWakeWordRunning();
    const bool has_error = HasLastErrorMessage();
    if (state != kDeviceStateIdle || has_error || !wake_word_running) {
        ESP_LOGI(TAG, "Startup wake word hint skipped (state=%d, wake word running=%d, error=%d)",
                 static_cast<int>(state), wake_word_running, has_error);
        return;
    }

    const auto wake_words = audio_service_.GetWakeWords();
    std::string words;
    for (auto it = wake_words.begin(); it != wake_words.end(); ++it) {
        if (it->empty() || std::find(wake_words.begin(), it, *it) != it) {
            continue;
        }
        if (!words.empty()) {
            words += " / ";
        }
        words += *it;
    }
    if (words.empty()) {
        ESP_LOGW(TAG, "Startup wake word hint skipped: no available wake words");
        return;
    }
    std::string message = Lang::Strings::WAKE_WORD_HINT;
    const auto placeholder = message.find("%s");
    if (placeholder == std::string::npos) {
        ESP_LOGE(TAG, "Startup wake word hint skipped: invalid message template");
        return;
    }
    message.replace(placeholder, 2, words);
    ESP_LOGI(TAG, "Startup wake word hint: %s", message.c_str());
    LogMemorySnapshot("before_startup_announcement");
    // Use the same bounded 12800 announcement API as touch wake and Web test.
    // The server owns its audio-synced captions; do not duplicate them locally.
    if (dedicated_startup && RequestAnnouncement("startup", message.c_str())) {
        ESP_LOGI(TAG, "[STARTUP] spoken announcement queued via WebSocket");
        return;
    }
    // No transport available: retain the original five-second local hint.
    // Use a separate message role only so timed removal cannot erase a newer
    // conversation subtitle.
    display->SetChatMessage(dedicated_startup ? "startup_hint" : "assistant", message.c_str());
    startup_wake_word_hint_message_ = std::move(message);
    if (startup_wake_word_hint_timer_ == nullptr) {
        esp_timer_create_args_t timer_args = {
            .callback =
                [](void* arg) {
                    auto app = static_cast<Application*>(arg);
                    xEventGroupSetBits(app->event_group_, MAIN_EVENT_STARTUP_HINT_TIMEOUT);
                },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "startup_hint",
            .skip_unhandled_events = false,
        };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &startup_wake_word_hint_timer_));
    }
    ESP_ERROR_CHECK(esp_timer_start_once(startup_wake_word_hint_timer_, 5000 * 1000));
}

void Application::HideStartupWakeWordHint() {
    if (startup_wake_word_hint_message_.empty()) {
        return;
    }
    // A conversation or alert may have replaced the hint while its timer was running.
    auto display = Board::GetInstance().GetDisplay();
    display->RemoveChatMessage(display->SupportsStartupSpeech() ? "startup_hint" : "assistant",
                               startup_wake_word_hint_message_.c_str());
    startup_wake_word_hint_message_.clear();
}

void Application::StartListeningAudio() {
    // Runs in the main loop, either directly from HandleStateChangedEvent or
    // deferred via MAIN_EVENT_PLAYBACK_DRAINED once the playback queue drains.
    if (GetDeviceState() != kDeviceStateListening) {
        return;
    }

    // Send the start listening command
    protocol_->SendStartListening(listening_mode_);
    audio_service_.EnableVoiceProcessing(true);

    ConfigureWakeWordForListening();

    // Play popup sound after ResetDecoder (in EnableVoiceProcessing) has been called
    if (play_popup_on_listening_) {
        play_popup_on_listening_ = false;
        audio_service_.PlaySound(Lang::Sounds::OGG_POPUP);
    }
}

void Application::ConfigureWakeWordForListening() {
#ifdef CONFIG_WAKE_WORD_DETECTION_IN_LISTENING
    // Enable wake word detection in listening mode (configured via Kconfig)
    audio_service_.EnableWakeWordDetection(!low_battery_mode_.load() && audio_service_.IsAfeWakeWord());
#else
    // Disable wake word detection in listening mode
    audio_service_.EnableWakeWordDetection(false);
#endif
}

void Application::StartNotification(std::string audio_url, std::vector<NotifySubtitle> subtitles) {
    if (GetDeviceState() != kDeviceStateIdle || notify_player_.IsBusy()) {
        ESP_LOGW(TAG, "Ignoring notify message while device is busy");
        return;
    }

    auto& board = Board::GetInstance();
    board.SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
    audio_service_.EnableVoiceProcessing(false);
    audio_service_.EnableWakeWordDetection(!low_battery_mode_.load() && audio_service_.IsAfeWakeWord());
    audio_service_.ReleaseWakeWordResources();
    while (audio_service_.PopPacketFromSendQueue()) {
        // Discard microphone audio left over from a previous conversation.
    }

    if (!SetDeviceState(kDeviceStateNotifying)) {
        board.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
        return;
    }

    audio_service_.ResetDecoder();
    uint32_t playback_id = ++notification_playback_id_;
    if (playback_id == 0) {
        playback_id = ++notification_playback_id_;
    }
    audio_service_.PlaySound(Lang::Sounds::OGG_POPUP);

    bool started = notify_player_.Start(
        std::move(audio_url), std::move(subtitles), playback_id,
        [this](uint32_t id, const std::string& text) {
            Schedule([this, id, text]() {
                if (GetDeviceState() == kDeviceStateNotifying && notification_playback_id_ == id) {
                    Board::GetInstance().GetDisplay()->SetChatMessage("assistant", text.c_str());
                }
            });
        },
        [this](uint32_t id, bool success) {
            Schedule([this, id, success]() { HandleNotificationFinished(id, success); });
        });

    if (!started) {
        ESP_LOGE(TAG, "Failed to start notification playback");
        StopNotification();
    }
}

void Application::StopNotification() {
    notify_player_.Stop();
    audio_service_.ResetDecoder();
    auto& board = Board::GetInstance();
    board.GetDisplay()->SetChatMessage("assistant", "");
    board.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
    if (GetDeviceState() == kDeviceStateNotifying) {
        SetDeviceState(kDeviceStateIdle);
    }
}

void Application::HandleNotificationFinished(uint32_t playback_id, bool success) {
    if (GetDeviceState() != kDeviceStateNotifying || notification_playback_id_ != playback_id) {
        return;
    }
    ESP_LOGI(TAG, "Notification playback %lu %s", static_cast<unsigned long>(playback_id),
             success ? "completed" : "failed");
    StopNotification();
}


const char* Application::AnnouncementTag(AnnouncementKind kind) {
    switch (kind) {
    case AnnouncementKind::Touch: return "touch";
    case AnnouncementKind::Test: return "test";
    case AnnouncementKind::Startup: return "startup";
    case AnnouncementKind::Morning: return "morning";
    case AnnouncementKind::Bedtime: return "bedtime";
    case AnnouncementKind::WakeWord: return "wake_word";
    case AnnouncementKind::Timer: return "timer";
    case AnnouncementKind::Snore: return "snore";
    case AnnouncementKind::SleepTalk: return "sleep_talk";
    case AnnouncementKind::DreamRemark: return "dream_remark";
    case AnnouncementKind::Monologue:
    case AnnouncementKind::MonologueTest: return "monologue";
    case AnnouncementKind::LowBattery: return "low_battery";
    case AnnouncementKind::DrowsyReply: return "drowsy_reply";
    case AnnouncementKind::ConversationRemark: return "conversation_remark";
    case AnnouncementKind::InterruptReply: return "interrupt_reply";
    default: return "none";
    }
}

bool Application::SetLowBatteryMode(bool active) {
    if (low_battery_mode_.exchange(active) == active) return true;
    low_battery_prepared_.store(false);
    if (!Schedule([this, active]() {
        if (low_battery_mode_.load() != active) return;
        if (active) {
            auto_reconnect_pending_ = false;
            wake_word_handoff_pending_ = false;
            pending_wake_word_greeting_.clear();
            wake_word_after_announcement_.clear();
            pending_listening_start_ = false;
            audio_service_.EnableWakeWordDetection(false);
            const auto state = GetDeviceState();
            if (IsAnnouncementActive()) {
                if (protocol_ && protocol_->IsAudioChannelOpened())
                    protocol_->SendAbortSpeaking(kAbortReasonNone);
                audio_service_.ResetDecoder();
                FinishAnnouncement("canceled_low_battery");
            } else if (state == kDeviceStateSpeaking || state == kDeviceStateListening ||
                       state == kDeviceStateConnecting) {
                if (protocol_ && protocol_->IsAudioChannelOpened())
                    protocol_->SendAbortSpeaking(kAbortReasonNone);
                audio_service_.ResetDecoder();
                CloseQuietConversation();
            } else if (state == kDeviceStateNotifying) {
                StopNotification();
            }
            low_battery_prepared_.store(true);
        } else {
            if (announcement_kind_.load() == AnnouncementKind::LowBattery) {
                if (protocol_ && protocol_->IsAudioChannelOpened())
                    protocol_->SendAbortSpeaking(kAbortReasonNone);
                audio_service_.ResetDecoder();
                FinishAnnouncement("canceled_external_power");
            }
            if (GetDeviceState() == kDeviceStateIdle)
                audio_service_.EnableWakeWordDetection(true);
        }
    })) {
        low_battery_mode_.store(!active);
        return false;
    }
    return true;
}

bool Application::RequestAnnouncement(const char* kind, const char* text, bool manual) {
    // All standalone sources share one audio-transport admission gate.
    // Never let an announcement reset a decoder while a chime, sound effect,
    // conversation, or another announcement is still playing.
    if (!kind || !text || !text[0] || std::strlen(text) > 512 ||
        !is_websocket_protocol_ || !protocol_ ||
        GetDeviceState() != kDeviceStateIdle || notify_player_.IsBusy() ||
        !audio_service_.IsPlaybackIdle()) {
        return false;
    }

    const bool low_battery = std::strcmp(kind, "low_battery") == 0;
    if (low_battery_mode_.load() != low_battery ||
        (low_battery && !low_battery_prepared_.load())) return false;
    AnnouncementKind requested = AnnouncementKind::None;
    if (low_battery) requested = AnnouncementKind::LowBattery;
    else if (std::strcmp(kind, "touch") == 0) requested = AnnouncementKind::Touch;
    else if (std::strcmp(kind, "test") == 0) requested = AnnouncementKind::Test;
    else if (std::strcmp(kind, "startup") == 0) requested = AnnouncementKind::Startup;
    else if (std::strcmp(kind, "morning") == 0) requested = AnnouncementKind::Morning;
    else if (std::strcmp(kind, "bedtime") == 0) requested = AnnouncementKind::Bedtime;
    else if (std::strcmp(kind, "wake_word") == 0) requested = AnnouncementKind::WakeWord;
    else if (std::strcmp(kind, "timer") == 0) requested = AnnouncementKind::Timer;
    else if (std::strcmp(kind, "snore") == 0) requested = AnnouncementKind::Snore;
    else if (std::strcmp(kind, "sleep_talk") == 0) requested = AnnouncementKind::SleepTalk;
    else if (std::strcmp(kind, "dream_remark") == 0) requested = AnnouncementKind::DreamRemark;
    else if (std::strcmp(kind, "drowsy_reply") == 0) requested = AnnouncementKind::DrowsyReply;
    else if (std::strcmp(kind, "conversation_remark") == 0) requested = AnnouncementKind::ConversationRemark;
    else if (std::strcmp(kind, "interrupt_reply") == 0) requested = AnnouncementKind::InterruptReply;
    else if (std::strcmp(kind, "monologue") == 0) requested = manual ? AnnouncementKind::MonologueTest : AnnouncementKind::Monologue;
    else return false;

    // One atomic admission operation protects every standalone speech source.
    AnnouncementKind expected = AnnouncementKind::None;
    if (!announcement_kind_.compare_exchange_strong(expected, requested)) return false;
    ESP_LOGI(TAG, "[SPEECH] kind=%s phase=queued transport=ws12800", kind);

    if (!Schedule([this, requested, kind = std::string(kind), message = std::string(text)]() {
        if (announcement_kind_.load() != requested) return;
        if (low_battery_mode_.load() != (requested == AnnouncementKind::LowBattery)) {
            FinishAnnouncement("canceled_mode_changed");
            return;
        }
        if (!protocol_ || !is_websocket_protocol_ || GetDeviceState() != kDeviceStateIdle ||
            !SetDeviceState(kDeviceStateConnecting)) {
            announcement_kind_.store(AnnouncementKind::None);
            return;
        }
        announcement_started_us_ = esp_timer_get_time();
        // Match the regular conversation path: wake Wi-Fi before opening the
        // socket, including standalone sleep talk and snoring.
        Board::GetInstance().SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
        // Publish CONNECTING before the potentially blocking WebSocket open.
        if (!Schedule([this, requested, kind, message]() {
            if (announcement_kind_.load() != requested) return;
            if (low_battery_mode_.load() != (requested == AnnouncementKind::LowBattery)) {
                FinishAnnouncement("canceled_mode_changed");
                return;
            }
            if (!protocol_ ||
                GetDeviceState() != kDeviceStateConnecting) {
                announcement_kind_.store(AnnouncementKind::None);
                return;
            }
            LogMemorySnapshot("before_announcement_ws");
            if ((!protocol_->IsAudioChannelOpened() && !protocol_->OpenAudioChannel()) ||
                !protocol_->SendAnnouncement(kind, message)) {
                LogMemorySnapshot("announcement_ws_failed");
                ESP_LOGW(TAG, "[SPEECH] kind=%s phase=send_failed transport=ws12800", kind.c_str());
                FinishAnnouncement("send_failed");
                return;
            }
            LogMemorySnapshot("announcement_ws_sent");
            ESP_LOGI(TAG, "[SPEECH] kind=%s phase=sent transport=ws12800 session=%s",
                     kind.c_str(), protocol_->session_id().c_str());
        })) FinishAnnouncement("queue_failed");
    })) {
        auto expected_kind = requested;
        announcement_kind_.compare_exchange_strong(expected_kind, AnnouncementKind::None);
        return false;
    }
    return true;
}

void Application::FinishAnnouncement(const char* result) {
    const auto completed = announcement_kind_.exchange(AnnouncementKind::None);
    if (completed == AnnouncementKind::None) return;
    ESP_LOGI(TAG, "[SPEECH] kind=%s phase=%s transport=ws12800",
             AnnouncementTag(completed),
             std::strcmp(result, "completed") == 0 ? "playback_completed" : result);
    announcement_started_us_ = 0;
    tts_stop_pending_ = false;
    announcement_subtitles_linger_ = true;
    if (protocol_) protocol_->CloseAudioChannel();
    Board::GetInstance().SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
    if (GetDeviceState() != kDeviceStateIdle) SetDeviceState(kDeviceStateIdle);
    // Wake-word handoff is deferred to the clock tick so no late close
    // callback from this socket can reset the newly opened conversation.
}

uint8_t Application::ActiveSleepAnnouncement() const {
    if (GetDeviceState() != kDeviceStateSpeaking) return 0;
    const auto kind = announcement_kind_.load();
    return kind == AnnouncementKind::Snore ? 1 :
           kind == AnnouncementKind::SleepTalk ? 2 : 0;
}

bool Application::IsQuietBackground(AnnouncementKind kind) {
    return kind == AnnouncementKind::Snore || kind == AnnouncementKind::SleepTalk ||
           kind == AnnouncementKind::DreamRemark || kind == AnnouncementKind::Monologue ||
           kind == AnnouncementKind::ConversationRemark || kind == AnnouncementKind::DrowsyReply ||
           kind == AnnouncementKind::Timer;
}

void Application::CloseQuietConversation() {
    quiet_end_pending_ = false;
    auto_reconnect_pending_ = false;
    pending_listening_start_ = false;
    ESP_LOGI(TAG, "[QUIET] current conversation ended");
    if (protocol_) protocol_->CloseAudioChannel();
    SetDeviceState(kDeviceStateIdle);
}

void Application::FinishConversationPlayback() {
    if (quiet_end_pending_) {
        CloseQuietConversation();
    } else {
        SetDeviceState(listening_mode_ == kListeningModeManualStop
                           ? kDeviceStateIdle : kDeviceStateListening);
    }
}

void Application::QuietModeStarted(bool voice_command) {
    Schedule([this, voice_command]() {
        const auto kind = announcement_kind_.load();
        if (IsQuietBackground(kind)) {
            if (protocol_ && protocol_->IsAudioChannelOpened())
                protocol_->SendAbortSpeaking(kAbortReasonNone);
            audio_service_.ResetDecoder();
            FinishAnnouncement("canceled_quiet");
            return;
        }
        if (kind != AnnouncementKind::None) return;
        const auto state = GetDeviceState();
        if (state == kDeviceStateSpeaking || (voice_command && state == kDeviceStateListening)) {
            quiet_end_pending_ = true;
            ESP_LOGI(TAG, "[QUIET] end conversation after playback");
        } else if (state == kDeviceStateListening || state == kDeviceStateConnecting) {
            CloseQuietConversation();
        } else if (state == kDeviceStateIdle) {
            auto_reconnect_pending_ = false;
        }
    });
}

void Application::CancelSleepAnnouncement() {
    // Invoked by StackChan from its UI task. Only sleep sounds can be
    // interrupted; never reset normal microphone conversations or greetings.
    const auto kind = announcement_kind_.load();
    if (kind != AnnouncementKind::Snore && kind != AnnouncementKind::SleepTalk) return;
    Schedule([this]() {
        const auto current = announcement_kind_.load();
        if (current != AnnouncementKind::Snore && current != AnnouncementKind::SleepTalk) return;
        if (protocol_ && protocol_->IsAudioChannelOpened())
            protocol_->SendAbortSpeaking(kAbortReasonNone);
        audio_service_.ResetDecoder(); // Only the standalone sleep sound.
        FinishAnnouncement("canceled");
    });
}

bool Application::Schedule(std::function<void()>&& callback) {
    static constexpr size_t kMaxMainTasks = 128;
    bool queued = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (main_tasks_.size() < kMaxMainTasks) {
            main_tasks_.push_back(std::move(callback));
            queued = true;
        }
    }
    if (!queued) {
        const uint32_t dropped = dropped_main_tasks_.fetch_add(1) + 1;
        if (dropped == 1 || dropped % 32 == 0) {
            ESP_LOGE(TAG, "Main task queue full; dropped callback (total=%lu)",
                     static_cast<unsigned long>(dropped));
        }
        return false;
    }
    xEventGroupSetBits(event_group_, MAIN_EVENT_SCHEDULE);
    return true;
}

std::string Application::LastErrorMessage() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_message_;
}

bool Application::HasLastErrorMessage() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return !last_error_message_.empty();
}

void Application::SetLastErrorMessage(const std::string& message) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_message_ = message;
}

void Application::ClearLastErrorMessage() {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_message_.clear();
}

void Application::PrepareTtsStart() {
    std::lock_guard<std::mutex> lock(pending_tts_audio_mutex_);
    pending_tts_audio_.clear();
    tts_start_pending_ = true;
}

void Application::HandleIncomingTtsAudio(std::unique_ptr<AudioStreamPacket> packet) {
    if (!packet) return;
    std::lock_guard<std::mutex> lock(pending_tts_audio_mutex_);
    if (!tts_start_pending_) {
        if (GetDeviceState() == kDeviceStateSpeaking) {
            audio_service_.PushPacketToDecodeQueue(std::move(packet));
        }
        return;
    }

    static constexpr size_t kMaxPendingTtsPackets = 16;
    if (pending_tts_audio_.size() >= kMaxPendingTtsPackets) {
        ESP_LOGW(TAG, "Pending TTS audio full; dropping packet before speaking state");
        return;
    }
    pending_tts_audio_.push_back(std::move(packet));
}

void Application::ActivateTtsPlayback() {
    std::lock_guard<std::mutex> lock(pending_tts_audio_mutex_);
    if (GetDeviceState() != kDeviceStateSpeaking) {
        if (!SetDeviceState(kDeviceStateSpeaking)) {
            tts_start_pending_ = false;
            pending_tts_audio_.clear();
            return;
        }
        // Reset before releasing buffered audio. The deferred state handler
        // runs later and would otherwise discard the first packets.
        audio_service_.ResetDecoder();
    }
    while (!pending_tts_audio_.empty()) {
        if (!audio_service_.PushPacketToDecodeQueue(std::move(pending_tts_audio_.front()))) {
            ESP_LOGW(TAG, "Decode queue full while flushing pending TTS audio");
        }
        pending_tts_audio_.pop_front();
    }
    tts_start_pending_ = false;
}

void Application::ClearPendingTtsAudio() {
    std::lock_guard<std::mutex> lock(pending_tts_audio_mutex_);
    tts_start_pending_ = false;
    pending_tts_audio_.clear();
}

void Application::AbortSpeaking(AbortReason reason) {
    ESP_LOGI(TAG, "Abort speaking");
    aborted_ = true;
    if (protocol_) {
        protocol_->SendAbortSpeaking(reason);
    }
}

void Application::SetListeningMode(ListeningMode mode) {
    listening_mode_ = mode;
    SetDeviceState(kDeviceStateListening);
}

ListeningMode Application::GetDefaultListeningMode() const {
    return aec_mode_ == kAecOff ? kListeningModeAutoStop : kListeningModeRealtime;
}

void Application::Reboot() {
    ESP_LOGI(TAG, "Rebooting...");
    if (GetDeviceState() == kDeviceStateNotifying) {
        StopNotification();
    }
    // Disconnect the audio channel
    if (protocol_ && protocol_->IsAudioChannelOpened()) {
        protocol_->CloseAudioChannel();
    }
    protocol_.reset();
    audio_service_.Stop();

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

bool Application::UpgradeFirmware(const std::string& url, const std::string& version) {
#if CONFIG_BOARD_TYPE_M5STACK_STACK_CHAN
    (void)url;
    (void)version;
    ESP_LOGW(TAG, "Firmware download is disabled on StackChan");
    return false;
#else
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();

    std::string upgrade_url = url;
    std::string version_info = version.empty() ? "(Manual upgrade)" : version;

    if (GetDeviceState() == kDeviceStateNotifying) {
        StopNotification();
    }

    // Close audio channel if it's open
    if (protocol_ && protocol_->IsAudioChannelOpened()) {
        ESP_LOGI(TAG, "Closing audio channel before firmware upgrade");
        protocol_->CloseAudioChannel();
    }
    ESP_LOGI(TAG, "Starting firmware upgrade from URL: %s", upgrade_url.c_str());

    Alert(Lang::Strings::OTA_UPGRADE, Lang::Strings::UPGRADING, "download",
          Lang::Sounds::OGG_UPGRADE);
    vTaskDelay(pdMS_TO_TICKS(3000));

    SetDeviceState(kDeviceStateUpgrading);

    std::string message = std::string(Lang::Strings::NEW_VERSION) + version_info;
    display->SetChatMessage("system", message.c_str());

    board.SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
    audio_service_.Stop();
    vTaskDelay(pdMS_TO_TICKS(1000));

    bool upgrade_success = Ota::Upgrade(upgrade_url, [this, display](int progress, size_t speed) {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "%d%% %uKB/s", progress, speed / 1024);
        Schedule([display, message = std::string(buffer)]() {
            display->SetChatMessage("system", message.c_str());
        });
    });

    if (!upgrade_success) {
        // Upgrade failed, restart audio service and continue running
        ESP_LOGE(TAG,
                 "Firmware upgrade failed, restarting audio service and continuing operation...");
        audio_service_.Start();                              // Restart audio service
        board.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);  // Restore power save level
        Alert(Lang::Strings::ERROR, Lang::Strings::UPGRADE_FAILED, "cancel",
              Lang::Sounds::OGG_EXCLAMATION);
        vTaskDelay(pdMS_TO_TICKS(3000));
        return false;
    } else {
        // Upgrade success, reboot immediately
        ESP_LOGI(TAG, "Firmware upgrade successful, rebooting...");
        display->SetChatMessage("system", "Upgrade successful, rebooting...");
        vTaskDelay(pdMS_TO_TICKS(1000));  // Brief pause to show message
        Reboot();
        return true;
    }
#endif
}

void Application::WakeWordInvoke(const std::string& wake_word) {
    if (low_battery_mode_.load()) return;
    if (!protocol_) {
        return;
    }

    auto state = GetDeviceState();

    if (state == kDeviceStateIdle) {
        // May be called from outside the main task (e.g. board button
        // callbacks), so schedule the invocation instead of running it here
        Schedule([this, wake_word]() {
            if (GetDeviceState() == kDeviceStateIdle) {
                BeginWakeWordInvoke(wake_word);
            }
        });
    } else if (state == kDeviceStateNotifying) {
        Schedule([this, wake_word]() {
            if (GetDeviceState() == kDeviceStateNotifying) {
                StopNotification();
                BeginWakeWordInvoke(wake_word);
            }
        });
    } else if (state == kDeviceStateSpeaking) {
        Schedule([this]() { AbortSpeaking(kAbortReasonNone); });
    } else if (state == kDeviceStateListening) {
        Schedule([this]() {
            if (protocol_) {
                protocol_->CloseAudioChannel();
            }
        });
    }
}

bool Application::CanEnterSleepMode() {
    if (GetDeviceState() != kDeviceStateIdle) {
        return false;
    }

    if (protocol_ && protocol_->IsAudioChannelOpened()) {
        return false;
    }

    if (!audio_service_.IsIdle()) {
        return false;
    }

    // Now it is safe to enter sleep mode
    return true;
}

void Application::RegisterMcpBroadcastCallback(std::function<void(const std::string&)> callback) {
    mcp_broadcast_callback_ = std::move(callback);
}

void Application::SendMcpMessage(const std::string& payload) {
    // Always schedule to run in main task for thread safety
    Schedule([this, payload]() {
        if (protocol_) {
            protocol_->SendMcpMessage(payload);
        }
        if (mcp_broadcast_callback_) {
            mcp_broadcast_callback_(payload);
        }
    });
}

void Application::SetAecMode(AecMode mode) {
    aec_mode_ = mode;
    Schedule([this]() {
        auto& board = Board::GetInstance();
        auto display = board.GetDisplay();
        switch (aec_mode_) {
            case kAecOff:
                audio_service_.EnableDeviceAec(false);
                display->ShowNotification(Lang::Strings::RTC_MODE_OFF);
                break;
            case kAecOnServerSide:
                audio_service_.EnableDeviceAec(false);
                display->ShowNotification(Lang::Strings::RTC_MODE_ON);
                break;
            case kAecOnDeviceSide:
                audio_service_.EnableDeviceAec(true);
                display->ShowNotification(Lang::Strings::RTC_MODE_ON);
                break;
        }

        // If the AEC mode is changed, close the audio channel
        if (protocol_ && protocol_->IsAudioChannelOpened()) {
            protocol_->CloseAudioChannel();
        }
    });
}

void Application::PlaySound(const std::string_view& sound) { audio_service_.PlaySound(sound); }

void Application::ResetProtocol() {
    Schedule([this]() {
        if (GetDeviceState() == kDeviceStateNotifying) {
            StopNotification();
        }
        // Close audio channel if opened
        if (protocol_ && protocol_->IsAudioChannelOpened()) {
            protocol_->CloseAudioChannel();
        }
        // Reset protocol
        auto_reconnect_pending_ = false;
        is_websocket_protocol_ = false;
        protocol_.reset();
    });
}
