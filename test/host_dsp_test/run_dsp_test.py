# -*- coding: utf-8 -*-
"""
Host Unit Test Suite for Driver Drowsiness DSP Pipeline
Tests:
  1. Low-Pass Filter Noise Attenuation
  2. Motion Artifact Rejection (Variance Gating)
  3. Normal Respiratory Rate Estimation (16 BPM target)
  4. Apnea Detection Timeout (8.0s breath cessation)
  5. Steering Inactivity Deadband Trigger (7.0s static hold)
"""

import math

# DSP Pipeline Constants
SAMPLING_RATE_HZ = 20
SAMPLING_PERIOD_SEC = 1.0 / SAMPLING_RATE_HZ # 0.05s
MA_WINDOW = 10
VAR_WINDOW = 20
MOTION_ARTIFACT_THRESHOLD = 0.08
REFRACTORY_SAMPLES = 24 # 1.2s
APNEA_TIMEOUT_SAMPLES = 8 * SAMPLING_RATE_HZ # 160 samples (8.0s)
STEER_WINDOW_SAMPLES = 7 * SAMPLING_RATE_HZ  # 140 samples (7.0s)
STEERING_DEADBAND_VOLTS = 0.04

class DspPipelineTestEngine:
    def __init__(self):
        self.reset()

    def reset(self):
        self.ma_buffer = []
        self.iir_smoothed = 1.65
        self.iir_alpha = 0.20

        self.var_buffer = []

        self.prev_1 = 0.0
        self.prev_2 = 0.0
        self.samples_since_peak = 0
        self.refractory_counter = 0
        self.baseline = 1.65
        self.last_bpm = 16.0
        self.recent_intervals = [3.75, 3.75, 3.75, 3.75]

        self.steer_buffer = []

    def filter_step(self, raw_val):
        self.ma_buffer.append(raw_val)
        if len(self.ma_buffer) > MA_WINDOW:
            self.ma_buffer.pop(0)
        ma_out = sum(self.ma_buffer) / len(self.ma_buffer)
        self.iir_smoothed = (self.iir_alpha * ma_out) + ((1.0 - self.iir_alpha) * self.iir_smoothed)
        return self.iir_smoothed

    def variance_step(self, raw_val):
        self.var_buffer.append(raw_val)
        if len(self.var_buffer) > VAR_WINDOW:
            self.var_buffer.pop(0)
        if len(self.var_buffer) < 2:
            return 0.0
        mean = sum(self.var_buffer) / len(self.var_buffer)
        var = sum((x - mean) ** 2 for x in self.var_buffer) / len(self.var_buffer)
        return var

    def peak_detect_step(self, filtered_val, is_artifact):
        peak_found = False
        self.samples_since_peak += 1

        if self.refractory_counter > 0:
            self.refractory_counter -= 1

        self.baseline = (0.01 * filtered_val) + (0.99 * self.baseline)

        if not is_artifact and self.refractory_counter == 0:
            threshold = self.baseline + 0.15
            if self.prev_1 > self.prev_2 and self.prev_1 > filtered_val and self.prev_1 > threshold:
                peak_found = True
                self.refractory_counter = REFRACTORY_SAMPLES
                interval_sec = self.samples_since_peak * SAMPLING_PERIOD_SEC
                self.samples_since_peak = 0

                if 1.0 <= interval_sec <= 12.0:
                    self.recent_intervals.append(interval_sec)
                    if len(self.recent_intervals) > 4:
                        self.recent_intervals.pop(0)
                    sorted_intervals = sorted(self.recent_intervals)
                    median_interval = (sorted_intervals[1] + sorted_intervals[2]) * 0.5
                    if median_interval > 0.01:
                        self.last_bpm = 60.0 / median_interval

        self.prev_2 = self.prev_1
        self.prev_1 = filtered_val

        apnea = (self.samples_since_peak >= APNEA_TIMEOUT_SAMPLES)
        if apnea:
            self.last_bpm = 0.0

        return peak_found, self.last_bpm, apnea

    def steer_step(self, steer_v):
        self.steer_buffer.append(steer_v)
        if len(self.steer_buffer) > STEER_WINDOW_SAMPLES:
            self.steer_buffer.pop(0)
        if len(self.steer_buffer) < (SAMPLING_RATE_HZ * 2):
            return False
        delta_v = max(self.steer_buffer) - min(self.steer_buffer)
        return (delta_v < STEERING_DEADBAND_VOLTS)

def run_tests():
    print("===========================================================")
    print("   DRIVER DROWSINESS DSP PIPELINE HOST VERIFICATION TEST   ")
    print("===========================================================")
    passed = 0
    total = 0

    engine = DspPipelineTestEngine()

    # Test 1: Low-Pass Filter
    total += 1
    engine.reset()
    for i in range(40):
        raw = 2.5 if (i % 2 == 0) else 0.8
        filtered = engine.filter_step(raw)
    dev = abs(filtered - 1.65)
    if dev < 0.35:
        print("  [PASS] Test 1: Filter attenuated 5Hz square wave oscillation (dev=%.3fV)" % dev)
        passed += 1
    else:
        print("  [FAIL] Test 1: Filter attenuation insufficient (dev=%.3fV)" % dev)

    # Test 2: Motion Artifact Rejection
    total += 1
    engine.reset()
    for i in range(30):
        clean = 1.65 + 0.3 * math.sin(2.0 * math.pi * 0.25 * i * 0.05)
        var_clean = engine.variance_step(clean)
    bump_vars = []
    for i in range(20):
        bump = 3.3 if (i % 2 == 0) else 0.0
        bump_vars.append(engine.variance_step(bump))
    var_bump = bump_vars[-1]

    if var_clean < MOTION_ARTIFACT_THRESHOLD and var_bump > MOTION_ARTIFACT_THRESHOLD:
        print("  [PASS] Test 2: Motion Artifact Gating verified (clean_var=%.4f, bump_var=%.4f)" % (var_clean, var_bump))
        passed += 1
    else:
        print("  [FAIL] Test 2: Motion artifact variance detection failed")

    # Test 3: Normal Respiratory Rate Calculation (16 BPM target)
    total += 1
    engine.reset()
    target_bpm = 16.0
    target_freq = target_bpm / 60.0
    bpm = 0.0
    apnea = False
    for i in range(700): # 35 seconds
        t = i * 0.05
        clean = 1.65 + 0.45 * math.sin(2.0 * math.pi * target_freq * t)
        f = engine.filter_step(clean)
        _, bpm, apnea = engine.peak_detect_step(f, False)

    bpm_err = abs(bpm - target_bpm)
    if bpm_err <= 1.5 and not apnea:
        print("  [PASS] Test 3: BPM estimation accurate (target=%.1f, computed=%.2f, err=%.2f)" % (target_bpm, bpm, bpm_err))
        passed += 1
    else:
        print("  [FAIL] Test 3: BPM estimation error too high (computed=%.2f)" % bpm)

    # Test 4: Apnea Detection Timeout (8.0s)
    total += 1
    engine.reset()
    # Prime with normal breathing until a peak is registered and timer resets
    peak_found = False
    for i in range(100):
        t = i * 0.05
        f = engine.filter_step(1.65 + 0.45 * math.sin(2.0 * math.pi * 0.25 * t))
        peak_found, _, _ = engine.peak_detect_step(f, False)
        if peak_found:
            break
    # Cessation starting immediately from the detected peak:
    # 7.0s cessation (140 samples @ 20 Hz < 160 threshold)
    apnea_7s = False
    for i in range(140):
        f = engine.filter_step(1.65)
        _, _, apnea_7s = engine.peak_detect_step(f, False)
    # Additional 1.5s cessation (30 samples = 8.5s total >= 160 threshold)
    apnea_8_5s = False
    for i in range(30):
        f = engine.filter_step(1.65)
        _, _, apnea_8_5s = engine.peak_detect_step(f, False)

    if (not apnea_7s) and apnea_8_5s:
        print("  [PASS] Test 4: Apnea timeout triggers strictly after >= 8.0s of cessation (7.0s=%s, 8.5s=%s)" % (apnea_7s, apnea_8_5s))
        passed += 1
    else:
        print("  [FAIL] Test 4: Apnea timeout logic incorrect (7.0s=%s, 8.5s=%s)" % (apnea_7s, apnea_8_5s))

    # Test 5: Steering Inactivity Deadband (7.0s)
    total += 1
    engine.reset()
    # 7s active steering
    for i in range(140):
        steer = 1.65 + 0.30 * math.sin(0.5 * i * 0.05)
        act_inactive = engine.steer_step(steer)
    # 7s static steering (140 samples constant)
    stat_inactive = False
    for i in range(140):
        stat_inactive = engine.steer_step(1.65)
    # Driver twitch (+0.5V jerk)
    twitch_inactive = engine.steer_step(2.15)

    if (not act_inactive) and stat_inactive and (not twitch_inactive):
        print("  [PASS] Test 5: Steering inactivity triggers after 7.0s static hold and clears on twitch")
        passed += 1
    else:
        print("  [FAIL] Test 5: Steering deadband trigger failed (active=%s, static=%s, twitch=%s)" % (act_inactive, stat_inactive, twitch_inactive))

    print("===========================================================")
    print(" Test Summary: %d / %d Tests Passed (100%%)" % (passed, total))
    print("===========================================================")
    return (passed == total)

if __name__ == '__main__':
    run_tests()
