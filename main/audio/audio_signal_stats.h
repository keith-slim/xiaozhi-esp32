#pragma once

#include <cstddef>
#include <cstdint>
#include <cmath>

// Task-local, allocation-free PCM diagnostics. Hash equality is an indicator
// (not proof) of repeated frames; count only adjacent frames of equal length.
class AudioSignalStats {
public:
    void Observe(const int16_t* data, size_t count, size_t stride = 1) {
        if (!data || !count || !stride) return;
        uint32_t hash = 2166136261u;
        uint32_t frame_peak = 0;
        size_t n = 0;
        for (size_t i = 0; i < count; i += stride) {
            const int32_t value = data[i];
            const uint32_t mag = value < 0 ? -value : value;
            square_sum_ += static_cast<uint64_t>(mag) * mag;
            if (mag > frame_peak) frame_peak = mag;
            hash = (hash ^ static_cast<uint16_t>(value)) * 16777619u;
            ++n;
        }
        samples_ += n;
        ++frames_;
        if (frame_peak > peak_) peak_ = frame_peak;
        if (frame_peak == 0) ++zero_frames_;
        if (frame_peak > 0 && has_previous_ && previous_hash_ == hash && previous_count_ == n)
            ++repeat_hash_frames_;
        previous_hash_ = hash;
        previous_count_ = n;
        has_previous_ = true;
    }

    uint32_t Frames() const { return frames_; }
    uint32_t Peak() const { return peak_; }
    uint32_t Rms() const {
        return samples_ ? static_cast<uint32_t>(std::sqrt(static_cast<double>(square_sum_) / samples_)) : 0;
    }
    uint32_t ZeroFrames() const { return zero_frames_; }
    uint32_t RepeatHashFrames() const { return repeat_hash_frames_; }
    void Reset() { *this = {}; }

private:
    uint64_t square_sum_ = 0;
    uint64_t samples_ = 0;
    size_t previous_count_ = 0;
    uint32_t frames_ = 0, peak_ = 0, zero_frames_ = 0, repeat_hash_frames_ = 0;
    uint32_t previous_hash_ = 0;
    bool has_previous_ = false;
};
