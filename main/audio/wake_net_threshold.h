#pragma once

// One setting shared by the AFE and the StackChan Web setup.
// 0 preserves the model's own threshold; other values are hundredths.
namespace WakeNetThreshold {
inline constexpr int kDefault = 0;
inline constexpr int kMin = 40;
inline constexpr int kMax = 90;
inline constexpr bool IsValid(int value) {
    return value == kDefault || (value >= kMin && value <= kMax);
}
int Get();
bool Save(int value);
}
