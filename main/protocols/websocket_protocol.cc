#include "websocket_protocol.h"
#include "application.h"
#include "board.h"
#include "settings.h"
#include "system_info.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <arpa/inet.h>
#include <cJSON.h>
#include <cstring>
#include <cerrno>
#include <lwip/sockets.h>
#include <lwip/opt.h>
#include <freertos/task.h>
#include "assets/lang_config.h"

// Optional StackChan-specific schedule hook. Other Xiaozhi boards remain unchanged.
extern "C" bool stackchan_wake_groggy_active() __attribute__((weak));

#define TAG "WS"

namespace {
// Read-only snapshot of lwIP descriptors; concurrent opens/closes may change
// the count during the scan. SO_TYPE does not consume pending socket errors.
void LogSocketUsage(const char* phase) {
#if defined(CONFIG_LWIP_MAX_SOCKETS) && defined(LWIP_SOCKET_OFFSET)
    const int saved_errno = errno;
    unsigned used = 0;
    unsigned unknown = 0;
    for (int fd = LWIP_SOCKET_OFFSET; fd < LWIP_SOCKET_OFFSET + CONFIG_LWIP_MAX_SOCKETS; ++fd) {
        int type = 0;
        socklen_t size = sizeof(type);
        if (getsockopt(fd, SOL_SOCKET, SO_TYPE, &type, &size) == 0) {
            ++used;
        } else if (errno != EBADF && errno != ENOTSOCK) {
            ++unknown;
        }
    }
    ESP_LOGI(TAG, "[WS_SOCKET_DIAG] phase=%s used=%u max=%u unknown=%u",
             phase, used, (unsigned)CONFIG_LWIP_MAX_SOCKETS, unknown);
    errno = saved_errno;
#endif
}
}  // namespace

WebsocketProtocol::WebsocketProtocol() { event_group_handle_ = xEventGroupCreate(); }

WebsocketProtocol::~WebsocketProtocol() {
    // Mark the transport as intentionally closed before its callbacks run.
    // This must happen while the protocol callback members are still alive.
    CloseAudioChannel();
    vEventGroupDelete(event_group_handle_);
}

bool WebsocketProtocol::Start() {
    // Only connect to server when audio channel is needed
    return true;
}

bool WebsocketProtocol::SendAudio(std::unique_ptr<AudioStreamPacket> packet) {
    if (websocket_ == nullptr || !websocket_->IsConnected()) {
        return false;
    }

    if (version_ == 2) {
        std::string serialized;
        serialized.resize(sizeof(BinaryProtocol2) + packet->payload.size());
        auto bp2 = (BinaryProtocol2*)serialized.data();
        bp2->version = htons(version_);
        bp2->type = 0;
        bp2->reserved = 0;
        bp2->timestamp = htonl(packet->timestamp);
        bp2->payload_size = htonl(packet->payload.size());
        memcpy(bp2->payload, packet->payload.data(), packet->payload.size());

        return websocket_->Send(serialized.data(), serialized.size(), true);
    } else if (version_ == 3) {
        std::string serialized;
        serialized.resize(sizeof(BinaryProtocol3) + packet->payload.size());
        auto bp3 = (BinaryProtocol3*)serialized.data();
        bp3->type = 0;
        bp3->reserved = 0;
        bp3->payload_size = htons(packet->payload.size());
        memcpy(bp3->payload, packet->payload.data(), packet->payload.size());

        return websocket_->Send(serialized.data(), serialized.size(), true);
    } else {
        return websocket_->Send(packet->payload.data(), packet->payload.size(), true);
    }
}

bool WebsocketProtocol::SendText(const std::string& text) {
    if (websocket_ == nullptr || !websocket_->IsConnected()) {
        return false;
    }

    if (!websocket_->Send(text)) {
        ESP_LOGE(TAG, "Failed to send text: %s", text.c_str());
        if (report_connect_errors_) {
            SetError(Lang::Strings::SERVER_ERROR);
        }
        return false;
    }

    return true;
}

bool WebsocketProtocol::IsAudioChannelOpened() const {
    return websocket_ != nullptr && websocket_->IsConnected() && !error_occurred_ && !IsTimeout();
}

void WebsocketProtocol::CloseAudioChannel(bool send_goodbye) {
    (void)send_goodbye;  // Websocket doesn't need to send goodbye message
    if (intentional_close_) {
        intentional_close_->store(true);
    }
    if (websocket_) {
        websocket_.reset();
        LogSocketUsage("after_close");
    }
}

bool WebsocketProtocol::OpenAudioChannel() {
    return OpenAudioChannelInternal(true);
}

bool WebsocketProtocol::ReconnectAudioChannel() {
    return OpenAudioChannelInternal(false);
}

bool WebsocketProtocol::OpenAudioChannelInternal(bool report_errors) {
    const int64_t connection_started_us = esp_timer_get_time();
    // Diagnostics only: never modify allocation, socket, or timeout behavior.
    const auto log_connect_memory = [connection_started_us](const char* phase) {
        ESP_LOGI(TAG, "[WS_MEM_DIAG] phase=%s elapsed_ms=%lu internal=%u largest=%u min_internal=%u psram=%u",
                 phase,
                 static_cast<unsigned long>((esp_timer_get_time() - connection_started_us) / 1000),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                 (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    };
    // Discard the previous transport before creating another one. Incrementing
    // the generation makes late callbacks from the old socket harmless.
    const unsigned generation = websocket_generation_.fetch_add(1) + 1;
    if (intentional_close_) {
        intentional_close_->store(true);
    }
    websocket_.reset();
    report_connect_errors_ = report_errors;

    Settings settings("websocket", false);
    std::string url = settings.GetString("url");
    std::string token = settings.GetString("token");
    int version = settings.GetInt("version");
    if (version != 0) {
        version_ = version;
    }

    error_occurred_ = false;
    log_connect_memory("before_socket_create");
    LogSocketUsage("before_connect");

    auto network = Board::GetInstance().GetNetwork();
    websocket_ = network->CreateWebSocket(1);
    log_connect_memory(websocket_ ? "socket_created" : "socket_create_failed");
    if (websocket_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create websocket");
        return false;
    }
    intentional_close_ = std::make_shared<std::atomic<bool>>(false);
    auto intentional_close = intentional_close_;
    auto disconnect_notified = std::make_shared<std::atomic<bool>>(false);

    if (!token.empty()) {
        // If token not has a space, add "Bearer " prefix
        if (token.find(" ") == std::string::npos) {
            token = "Bearer " + token;
        }
        websocket_->SetHeader("Authorization", token.c_str());
    }
    websocket_->SetHeader("Protocol-Version", std::to_string(version_).c_str());
    websocket_->SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
    websocket_->SetHeader("Client-Id", Board::GetInstance().GetUuid().c_str());
    // Context is evaluated afresh for every WebSocket connection (normal
    // conversation and each standalone speech event share this path).
    if (stackchan_wake_groggy_active && stackchan_wake_groggy_active()) {
        websocket_->SetHeader("X-StackChan-Wake-Groggy", "1");
        ESP_LOGI(TAG, "[SPEECH] wake_groggy=1");
    }
    log_connect_memory("headers_ready");

    websocket_->OnData([this, generation, intentional_close, connection_started_us, first_frame = true](const char* data, size_t len, bool binary) mutable {
        if (generation != websocket_generation_.load()) return;
        if (first_frame) {
            first_frame = false;
            ESP_LOGI(TAG, "[WS_RX_DIAG] first_frame elapsed_ms=%lu bytes=%u binary=%u internal=%u largest=%u",
                     (unsigned long)((esp_timer_get_time() - connection_started_us) / 1000),
                     (unsigned)len, (unsigned)binary,
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
#if (INCLUDE_uxTaskGetStackHighWaterMark == 1)
            ESP_LOGI(TAG, "[WS_RX_DIAG] receive_task=%s stack_free=%u",
                     pcTaskGetName(nullptr), (unsigned)uxTaskGetStackHighWaterMark(nullptr));
#endif
        }
        if (binary) {
            if (on_incoming_audio_ != nullptr) {
                // Validate the framing before constructing a PCM/Opus packet.
                // A truncated WebSocket frame must never read beyond 'len'.
                const uint8_t* payload = reinterpret_cast<const uint8_t*>(data);
                size_t payload_length = len;
                uint32_t timestamp = 0;
                if (version_ == 2) {
                    if (len < sizeof(BinaryProtocol2)) {
                        ESP_LOGW(TAG, "Truncated protocol-v2 audio header: %u bytes", (unsigned)len);
                        return;
                    }
                    const auto* header = reinterpret_cast<const BinaryProtocol2*>(data);
                    payload_length = ntohl(header->payload_size);
                    if (payload_length > len - sizeof(BinaryProtocol2)) {
                        ESP_LOGW(TAG, "Invalid protocol-v2 audio payload length");
                        return;
                    }
                    payload = header->payload;
                    timestamp = ntohl(header->timestamp);
                } else if (version_ == 3) {
                    if (len < sizeof(BinaryProtocol3)) {
                        ESP_LOGW(TAG, "Truncated protocol-v3 audio header: %u bytes", (unsigned)len);
                        return;
                    }
                    const auto* header = reinterpret_cast<const BinaryProtocol3*>(data);
                    payload_length = ntohs(header->payload_size);
                    if (payload_length > len - sizeof(BinaryProtocol3)) {
                        ESP_LOGW(TAG, "Invalid protocol-v3 audio payload length");
                        return;
                    }
                    payload = header->payload;
                }
                on_incoming_audio_(std::make_unique<AudioStreamPacket>(AudioStreamPacket{
                    .sample_rate = server_sample_rate_,
                    .frame_duration = server_frame_duration_,
                    .timestamp = timestamp,
                    .payload = std::vector<uint8_t>(payload, payload + payload_length)}));
            }
        } else {
            // Parse JSON data
            auto root = cJSON_ParseWithLength(data, len);
            auto type = cJSON_GetObjectItem(root, "type");
            if (cJSON_IsString(type)) {
                if (strcmp(type->valuestring, "hello") == 0) {
                    ParseServerHello(root);
                } else if (strcmp(type->valuestring, "session") == 0) {
                    // Only the explicit server idle timeout is a normal end
                    // of conversation. Provider and network failures still
                    // trigger automatic reconnect.
                    auto reason = cJSON_GetObjectItem(root, "reason");
                    if (cJSON_IsString(reason) &&
                        strcmp(reason->valuestring, "idle") == 0 &&
                        generation == websocket_generation_.load()) {
                        intentional_close->store(true);
                        ESP_LOGI(TAG, "Server ended idle session; returning to wake-word standby");
                    }
                } else {
                    if (on_incoming_json_ != nullptr) {
                        on_incoming_json_(root);
                    }
                }
            } else {
                ESP_LOGE(TAG, "Missing message type, data: %s", std::string(data, len).c_str());
            }
            cJSON_Delete(root);
        }
        MarkIncoming();
    });

    // Callbacks run on the transport's receive task, which on 4G modems is the
    // AT event task and cannot send, so the Pong is sent from the main task.
    std::weak_ptr<WebSocket> weak_websocket = websocket_;
    websocket_->OnPing([weak_websocket](const char* data, size_t len) {
        Application::GetInstance().Schedule([weak_websocket, payload = std::string(data, len)]() {
            if (auto websocket = weak_websocket.lock()) {
                websocket->SendPong(payload.data(), payload.size());
            }
        });
    });

    websocket_->OnDisconnected([this, generation, intentional_close, disconnect_notified]() {
        if (generation != websocket_generation_.load() || disconnect_notified->exchange(true)) {
            return;
        }
        const bool unexpected = !intentional_close->load();
        ESP_LOGI(TAG, "Websocket disconnected (%s)", unexpected ? "unexpected" : "requested");
        callback_generation_.store(generation, std::memory_order_relaxed);
        // Notify unexpected loss first; Application snapshots its active state
        // before the regular close callback transitions it back to idle.
        if (unexpected && on_disconnected_ != nullptr) {
            on_disconnected_();
        }
        if (on_audio_channel_closed_ != nullptr) {
            on_audio_channel_closed_();
        }
    });

    ESP_LOGI(TAG, "Connecting to websocket server: %s with version: %d", url.c_str(), version_);
    if (auto connected = websocket_->Connect(url.c_str()); !connected) {
        log_connect_memory("connect_failed");
        LogSocketUsage("connect_failed");
        ESP_LOGE(TAG, "Failed to connect to websocket server: %s",
                 connected.error().ToString().c_str());
        if (report_errors) {
            SetError(Lang::Strings::SERVER_NOT_CONNECTED, url);
        }
        return false;
    }

    log_connect_memory("websocket_connected");
    LogSocketUsage("websocket_connected");

    // Send hello message to describe the client
    auto message = GetHelloMessage();
    const int64_t hello_send_started_us = esp_timer_get_time();
    if (message.empty() || !SendText(message)) {
        ESP_LOGE(TAG, "Failed to create or send websocket hello");
        log_connect_memory("hello_send_failed");
        return false;
    }
    ESP_LOGI(TAG, "[WS_TX_DIAG] hello bytes=%u send_us=%lu", (unsigned)message.size(),
             (unsigned long)(esp_timer_get_time() - hello_send_started_us));
    log_connect_memory("hello_sent");

    // Wait for server hello
    EventBits_t bits =
        xEventGroupWaitBits(event_group_handle_, WEBSOCKET_PROTOCOL_SERVER_HELLO_EVENT, pdTRUE,
                            pdFALSE, pdMS_TO_TICKS(15000));
    log_connect_memory((bits & WEBSOCKET_PROTOCOL_SERVER_HELLO_EVENT) ? "hello_received" : "hello_timeout");
    if (!(bits & WEBSOCKET_PROTOCOL_SERVER_HELLO_EVENT)) {
        LogSocketUsage("hello_timeout");
        ESP_LOGE(TAG, "Failed to receive server hello");
        if (report_errors) {
            SetError(Lang::Strings::SERVER_TIMEOUT);
        }
        return false;
    }

    report_connect_errors_ = true;
    if (on_audio_channel_opened_ != nullptr) {
        callback_generation_.store(generation, std::memory_order_relaxed);
        on_audio_channel_opened_();
    }

    return true;
}

std::string WebsocketProtocol::GetHelloMessage() {
    // keys: message type, version, audio_params (format, sample_rate, channels)
    cJSON* root = cJSON_CreateObject();
    if (!root) return {};
    if (!cJSON_AddStringToObject(root, "type", "hello") ||
        !cJSON_AddNumberToObject(root, "version", version_)) {
        cJSON_Delete(root);
        return {};
    }
    cJSON* features = cJSON_CreateObject();
    if (!features) {
        cJSON_Delete(root);
        return {};
    }
#if CONFIG_USE_SERVER_AEC
    if (!cJSON_AddBoolToObject(features, "aec", true)) {
        cJSON_Delete(features);
        cJSON_Delete(root);
        return {};
    }
#endif
    if (!cJSON_AddBoolToObject(features, "mcp", true) ||
        !cJSON_AddItemToObject(root, "features", features)) {
        cJSON_Delete(features);
        cJSON_Delete(root);
        return {};
    }
    AddTextFontCapabilities(root);
    if (!cJSON_AddStringToObject(root, "transport", "websocket")) {
        cJSON_Delete(root);
        return {};
    }
    cJSON* audio_params = cJSON_CreateObject();
    if (!audio_params || !cJSON_AddStringToObject(audio_params, "format", "opus") ||
        !cJSON_AddNumberToObject(audio_params, "sample_rate", 16000) ||
        !cJSON_AddNumberToObject(audio_params, "channels", 1) ||
        !cJSON_AddNumberToObject(audio_params, "frame_duration", OPUS_FRAME_DURATION_MS) ||
        !cJSON_AddItemToObject(root, "audio_params", audio_params)) {
        cJSON_Delete(audio_params);
        cJSON_Delete(root);
        return {};
    }
    auto json_str = cJSON_PrintUnformatted(root);
    std::string message;
    if (json_str) {
        message.assign(json_str);
        cJSON_free(json_str);
    }
    cJSON_Delete(root);
    return message;
}

void WebsocketProtocol::ParseServerHello(const cJSON* root) {
    auto transport = cJSON_GetObjectItem(root, "transport");
    if (!cJSON_IsString(transport)) {
        ESP_LOGE(TAG, "Missing or non-string transport in server hello");
        return;
    }
    if (strcmp(transport->valuestring, "websocket") != 0) {
        ESP_LOGE(TAG, "Unsupported transport: %s", transport->valuestring);
        return;
    }

    auto session_id = cJSON_GetObjectItem(root, "session_id");
    if (cJSON_IsString(session_id)) {
        session_id_ = session_id->valuestring;
        ESP_LOGI(TAG, "Session ID: %s", session_id_.c_str());
    }

    auto audio_params = cJSON_GetObjectItem(root, "audio_params");
    if (cJSON_IsObject(audio_params)) {
        auto sample_rate = cJSON_GetObjectItem(audio_params, "sample_rate");
        if (cJSON_IsNumber(sample_rate)) {
            server_sample_rate_ = sample_rate->valueint;
        }
        auto frame_duration = cJSON_GetObjectItem(audio_params, "frame_duration");
        if (cJSON_IsNumber(frame_duration)) {
            server_frame_duration_ = frame_duration->valueint;
        }
    }

    xEventGroupSetBits(event_group_handle_, WEBSOCKET_PROTOCOL_SERVER_HELLO_EVENT);
}
