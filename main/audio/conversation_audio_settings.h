#pragma once

// Conversation-only digital input settings. The AFE instance is shared with
// WakeNet; never rebuild it on a live HTTP request.
namespace ConversationAudio {
inline constexpr int kGainMin = -12;
inline constexpr int kGainMax = 12;
inline constexpr int kVadDefault = 1; // Gemini default; 0=low, 2=high.
inline constexpr int kAgcMax = 4;

struct Settings {
    int gain_db = 0;
    int vad_mode = kVadDefault;
    int agc = 0; // Off; each step permits up to 3 dB additional gain.
};

inline constexpr int Pack(const Settings& s) {
    return (s.gain_db - kGainMin) + 32 * s.vad_mode + 256 * s.agc;
}
inline constexpr Settings Unpack(int n) {
    return {(n & 31) + kGainMin, (n >> 5) & 7, (n >> 8) & 7};
}
inline constexpr bool Valid(const Settings& s, int max_vad = 2) {
    return s.gain_db >= kGainMin && s.gain_db <= kGainMax &&
           s.vad_mode >= 0 && s.vad_mode <= max_vad &&
           s.agc >= 0 && s.agc <= kAgcMax;
}

Settings Get(); // Cached after the first read.
bool Save(const Settings& value); // Main task; one NVS commit.
}
