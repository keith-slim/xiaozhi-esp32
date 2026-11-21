#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <esp_timer.h>

#include <atomic>
#include <string>
#include <mutex>
#include <deque>
#include <memory>
#include <functional>
#include <utility>
#include <cstdint>
#include <vector>

#include "protocol.h"
#include "ota.h"
#include "audio_service.h"
#include "device_state.h"
#include "device_state_machine.h"
#include "notify/notify_player.h"

// Main event bits
#define MAIN_EVENT_SCHEDULE             (1 << 0)
#define MAIN_EVENT_SEND_AUDIO           (1 << 1)
#define MAIN_EVENT_WAKE_WORD_DETECTED   (1 << 2)
#define MAIN_EVENT_VAD_CHANGE           (1 << 3)
#define MAIN_EVENT_ERROR                (1 << 4)
#define MAIN_EVENT_ACTIVATION_DONE      (1 << 5)
#define MAIN_EVENT_CLOCK_TICK           (1 << 6)
#define MAIN_EVENT_NETWORK_CONNECTED    (1 << 7)
#define MAIN_EVENT_NETWORK_DISCONNECTED (1 << 8)
#define MAIN_EVENT_TOGGLE_CHAT          (1 << 9)
#define MAIN_EVENT_START_LISTENING      (1 << 10)
#define MAIN_EVENT_STOP_LISTENING       (1 << 11)
#define MAIN_EVENT_STATE_CHANGED        (1 << 12)
#define MAIN_EVENT_PLAYBACK_DRAINED     (1 << 13)
#define MAIN_EVENT_STARTUP_HINT_TIMEOUT (1 << 14)

enum AecMode {
    kAecOff,
    kAecOnDeviceSide,
    kAecOnServerSide,
};

class Application {
    // One atomic state represents both exclusive standalone speech ownership
    // and its origin. No separate startup lifecycle/lock is required.
    enum class AnnouncementKind : uint8_t {
        None, Touch, Test, Startup, Morning, Bedtime, WakeWord, Timer, Snore, SleepTalk,
        DreamRemark, Monologue, MonologueTest
    };

public:
    static Application& GetInstance() {
        static Application instance;
        return instance;
    }
    // Delete copy constructor and assignment operator
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    /**
     * Initialize the application
     * This sets up display, audio, network callbacks, etc.
     * Network connection starts asynchronously.
     */
    void Initialize();

    // Optional local service, installed before Initialize(). Called only from
    // the main event loop after network connection, before AI activation.
    // Return true once started; false permits a later retry.
    void SetLocalNetworkServiceStarter(bool (*starter)()) {
        local_network_service_starter_ = starter;
    }

    /**
     * Run the main event loop
     * This function runs in the main task and never returns.
     * It handles all events including network, state changes, and user interactions.
     */
    void Run();

    DeviceState GetDeviceState() const { return state_machine_.GetState(); }
    bool IsVoiceDetected() const { return audio_service_.IsVoiceDetected(); }
    
    /**
     * Request state transition
     * Returns true if transition was successful
     */
    bool SetDeviceState(DeviceState state);

    /**
     * Schedule a callback to be executed in the main task
     */
    bool Schedule(std::function<void()>&& callback);

    /**
     * Alert with status, message, emotion and optional sound
     */
    void Alert(const char* status, const char* message, const char* emotion = "", const std::string_view& sound = "");
    void DismissAlert();

    void AbortSpeaking(AbortReason reason);

    /**
     * Toggle chat state (event-based, thread-safe)
     * Sends MAIN_EVENT_TOGGLE_CHAT to be handled in Run()
     */
    void ToggleChatState();

    /**
     * Start listening (event-based, thread-safe)
     * Sends MAIN_EVENT_START_LISTENING to be handled in Run()
     */
    void StartListening();

    /**
     * Stop listening (event-based, thread-safe)
     * Sends MAIN_EVENT_STOP_LISTENING to be handled in Run()
     */
    void StopListening();

    void Reboot();
    void WakeWordInvoke(const std::string& wake_word);
    bool UpgradeFirmware(const std::string& url, const std::string& version = "");
    bool CanEnterSleepMode();
    void SendMcpMessage(const std::string& payload);
    void RegisterMcpBroadcastCallback(std::function<void(const std::string&)> callback);
    void SetAecMode(AecMode mode);
    AecMode GetAecMode() const { return aec_mode_; }
    void PlaySound(const std::string_view& sound);
    AudioService& GetAudioService() { return audio_service_; }
    // Opt-in text-only speech using the existing Xiaozhi WebSocket/decoder.
    // Nonblocking: no network work under the StackChan LVGL lock.
    bool RequestAnnouncement(const char* kind, const char* text, bool manual = false);
    // End only the current conversation at its playback boundary, and stop
    // autonomous speech without interrupting manual tests or greetings.
    void QuietModeStarted(bool voice_command = false);
    // Optional StackChan sleep-wake hook; called only on an actual wake-word event.
    // Empty result preserves the original immediate listening behavior.
    void SetWakeWordAnnouncementProvider(std::function<std::string()> provider) {
        wake_word_announcement_provider_ = std::move(provider);
    }
    void CancelSleepAnnouncement(); // Never interrupts an ordinary conversation.
    uint8_t ActiveSleepAnnouncement() const; // 0=none, 1=snore, 2=sleep talk; only during TTS.
    bool IsAnnouncementActive() const { return announcement_kind_.load() != AnnouncementKind::None; }
    bool IsStartupAnnouncementActive() const { return announcement_kind_.load() == AnnouncementKind::Startup; }

    
    /**
     * Reset protocol resources (thread-safe)
     * Can be called from any task to release resources allocated after network connected
     * This includes closing audio channel, resetting protocol and ota objects
     */
    void ResetProtocol();

private:
    Application();
    ~Application();

    std::mutex mutex_;
    std::deque<std::function<void()>> main_tasks_;
    std::atomic<uint32_t> dropped_main_tasks_{0};
    std::unique_ptr<Protocol> protocol_;
    EventGroupHandle_t event_group_ = nullptr;
    esp_timer_handle_t clock_timer_handle_ = nullptr;
    esp_timer_handle_t startup_wake_word_hint_timer_ = nullptr;
    DeviceStateMachine state_machine_;
    ListeningMode listening_mode_ = kListeningModeAutoStop;
    AecMode aec_mode_ = kAecOff;
    mutable std::mutex error_mutex_;
    std::string last_error_message_;
    std::string startup_wake_word_hint_message_;
    int64_t startup_hint_voice_wait_since_us_ = 0; // Main loop; bounded startup sound drain wait.
    AudioService audio_service_;
    std::atomic<uint32_t> last_speech_subtitle_tick_ms_{0};
    bool quiet_end_pending_ = false; // main task only; current conversation
    bool tts_stop_pending_ = false;  // Stay in speaking state until actual queued PCM drains.
    std::mutex pending_tts_audio_mutex_;
    std::deque<std::unique_ptr<AudioStreamPacket>> pending_tts_audio_;
    bool tts_start_pending_ = false;
    std::atomic<AnnouncementKind> announcement_kind_{AnnouncementKind::None};
    int64_t announcement_started_us_ = 0; // main task only, bounded timeout
    bool announcement_subtitles_linger_ = false; // main task only
    std::function<std::string()> wake_word_announcement_provider_;
    std::string wake_word_after_announcement_; // Main-task only, resumed after socket close.
    bool wake_word_handoff_pending_ = false; // An empty recognized word must still resume listening.
    std::string pending_wake_word_greeting_; // Used when wake word interrupts snore/sleep talk.
    int64_t pending_wake_word_since_us_ = 0; // Bounded fallback if the greeting cannot start.

    NotifyPlayer notify_player_;
    uint32_t notification_playback_id_ = 0;
    std::unique_ptr<Ota> ota_;

    std::function<void(const std::string&)> mcp_broadcast_callback_;

    bool has_server_time_ = false;
    bool startup_completed_ = false;
    bool pending_startup_wake_word_hint_ = false;
    bool aborted_ = false;
    bool assets_version_checked_ = false;
    bool play_popup_on_listening_ = false;  // Flag to play popup sound after state changes to listening
    bool pending_listening_start_ = false;  // Waiting for playback to drain before starting listening (auto mode)
    int clock_ticks_ = 0;
    TaskHandle_t activation_task_handle_ = nullptr;
    bool (*local_network_service_starter_)() = nullptr;
    bool local_network_connected_ = false; // main event loop only
    bool is_websocket_protocol_ = false;
    bool auto_reconnect_pending_ = false;
    unsigned auto_reconnect_attempts_ = 0;
    int64_t auto_reconnect_due_us_ = 0;


    // Event handlers
    void HandleStateChangedEvent();
    void HandleToggleChatEvent();
    void HandleStartListeningEvent();
    void HandleStopListeningEvent();
    void TryStartLocalNetworkService();
    void HandleNetworkConnectedEvent();
    void HandleNetworkDisconnectedEvent();
    void HandleActivationDoneEvent();
    void HandleWakeWordDetectedEvent();
    void ContinueOpenAudioChannel(ListeningMode mode);
    void MaybeReconnectAudioChannel();
    void FinishAnnouncement(const char* result = "completed");
    void FinishConversationPlayback();
    void CloseQuietConversation();
    static bool IsQuietBackground(AnnouncementKind kind);
    static const char* AnnouncementTag(AnnouncementKind kind);
    void BeginWakeWordInvoke(const std::string& wake_word);
    void ContinueWakeWordInvoke(const std::string& wake_word);
    void StartListeningAudio();
    void ConfigureWakeWordForListening();
    void ShowStartupWakeWordHint();
    void HideStartupWakeWordHint();
    void StartNotification(std::string audio_url, std::vector<NotifySubtitle> subtitles);
    void StopNotification();
    void HandleNotificationFinished(uint32_t playback_id, bool success);
    std::string LastErrorMessage() const;
    bool HasLastErrorMessage() const;
    void SetLastErrorMessage(const std::string& message);
    void ClearLastErrorMessage();
    void PrepareTtsStart();
    void HandleIncomingTtsAudio(std::unique_ptr<AudioStreamPacket> packet);
    void ActivateTtsPlayback();
    void ClearPendingTtsAudio();

    // Activation task (runs in background)
    void ActivationTask();

    // Helper methods
    void CheckAssetsVersion();
    void CheckNewVersion();
    void InitializeProtocol();
    void SetListeningMode(ListeningMode mode);
    ListeningMode GetDefaultListeningMode() const;
    
    // State change handler called by state machine
    void OnStateChanged(DeviceState old_state, DeviceState new_state);
};


class TaskPriorityReset {
public:
    TaskPriorityReset(BaseType_t priority) {
        original_priority_ = uxTaskPriorityGet(NULL);
        vTaskPrioritySet(NULL, priority);
    }
    ~TaskPriorityReset() {
        vTaskPrioritySet(NULL, original_priority_);
    }

private:
    BaseType_t original_priority_;
};

#endif // _APPLICATION_H_
