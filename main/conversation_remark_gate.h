#pragma once
#include <cstdint>
#include <string>
#include <utility>

// Only a server-authorized, fully drained ordinary TTS reply can arm this
// one-shot gate. Caller serializes access; times are monotonic milliseconds.
class ConversationRemarkGate {
public:
    void Clear() { session_.clear(); due_ = 0; }
    void ConfirmPlayback(std::string session) {
        if (session.empty()) return;
        session_ = std::move(session);
        due_ = 0;
    }
    void OnIdle(uint64_t now, uint32_t delay_ms) {
        if (!session_.empty() && !due_) due_ = now + delay_ms;
    }
    std::string Take(uint64_t now, bool eligible, bool quiet) {
        if (quiet || (due_ && now >= due_ + 45000)) {
            Clear();
            return {};
        }
        if (!eligible || !due_ || now < due_) return {};
        auto session = std::move(session_);
        Clear();
        return session;
    }
private:
    std::string session_;
    uint64_t due_ = 0;
};
