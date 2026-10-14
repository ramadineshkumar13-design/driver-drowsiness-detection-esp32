/**
 * @file test_dsp.c
 * @brief Host-side unit test harness for Driver Drowsiness DSP Pipeline
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>
#include "dsp_pipeline.h"

#define ANSI_GREEN  "\033[32m"
#define ANSI_RED    "\033[31m"
#define ANSI_RESET  "\033[0m"

static int s_tests_passed = 0;
static int s_tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    if (cond) { \
        printf("  [PASS] %s\n", msg); \
        s_tests_passed++; \
    } else { \
        printf("  %s[FAIL] %s%s\n", ANSI_RED, msg, ANSI_RESET); \
        s_tests_failed++; \
    } \
} while(0)

/* =========================================================================
 * Test 1: Low-Pass Filter Noise Attenuation
 * ========================================================================= */
static void test_filter_noise_attenuation(void)
{
    printf("\n--- Test 1: Filter High-Frequency Noise Attenuation ---\n");
    dsp_pipeline_reset();

    /* Feed 5 Hz noise square wave (frequency >> 0.5 Hz breathing) */
    float high_val = 2.5f;
    float low_val  = 0.8f;
    float filtered = 1.65f;

    for (int i = 0; i < 40; i++) {
        float raw = (i % 2 == 0) ? high_val : low_val;
        filtered = dsp_filter_step(raw);
    }

    /* Verify filter smoothly attenuated the 1.7V peak-to-peak oscillation down */
    float deviation = fabsf(filtered - 1.65f);
    TEST_ASSERT(deviation < 0.35f, "Filter successfully attenuated 5Hz square noise oscillation");
}

/* =========================================================================
 * Test 2: Motion Artifact Rejection (Road Bump Variance Gating)
 * ========================================================================= */
static void test_artifact_rejection(void)
{
    printf("\n--- Test 2: Motion Artifact Rejection (Variance Gating) ---\n");
    dsp_pipeline_reset();

    /* 1. Normal clean breathing signal variance */
    float var_normal = 0.0f;
    for (int i = 0; i < 30; i++) {
        float clean_sample = 1.65f + 0.3f * sinf(2.0f * (float)M_PI * 0.25f * (float)i * 0.05f);
        var_normal = dsp_variance_step(clean_sample);
    }
    TEST_ASSERT(var_normal < DSP_MOTION_ARTIFACT_THRESHOLD,
                "Clean respiration variance remains below artifact threshold");

    /* 2. Sudden road bump shock (large violent displacement) */
    float var_bump = 0.0f;
    for (int i = 0; i < 20; i++) {
        float bump_sample = (i % 2 == 0) ? 3.3f : 0.0f; // 3.3V shock
        var_bump = dsp_variance_step(bump_sample);
    }
    TEST_ASSERT(var_bump > DSP_MOTION_ARTIFACT_THRESHOLD,
                "Road bump shock triggers variance threshold (Artifact Gated)");
}

/* =========================================================================
 * Test 3: Normal Respiratory Rate Calculation (16 BPM)
 * ========================================================================= */
static void test_bpm_calculation(void)
{
    printf("\n--- Test 3: BPM Estimation Accuracy (Target 16 BPM) ---\n");
    dsp_pipeline_reset();

    float target_bpm = 16.0f;
    float target_freq = target_bpm / 60.0f; // 0.267 Hz
    float bpm = 0.0f;
    bool apnea = false;

    /* Simulate 35 seconds of clean 16 BPM breathing (700 samples @ 20 Hz) */
    for (int i = 0; i < 700; i++) {
        float t = (float)i * 0.05f;
        float clean_signal = 1.65f + 0.45f * sinf(2.0f * (float)M_PI * target_freq * t);
        float filtered = dsp_filter_step(clean_signal);
        dsp_peak_detect_step(filtered, false, &bpm, &apnea);
    }

    float bpm_error = fabsf(bpm - target_bpm);
    printf("  Estimated BPM: %.2f (Error: %.2f BPM)\n", bpm, bpm_error);
    TEST_ASSERT(bpm_error <= 1.5f, "Calculated BPM matches 16 BPM within +/- 1.5 BPM margin");
    TEST_ASSERT(!apnea, "Apnea flag remains FALSE during normal rhythmic breathing");
}

/* =========================================================================
 * Test 4: Apnea Detection Timeout (8 Seconds Breath Cessation)
 * ========================================================================= */
static void test_apnea_detection(void)
{
    printf("\n--- Test 4: Apnea Detection Timeout (8.0s Cessation) ---\n");
    dsp_pipeline_reset();

    float bpm = 0.0f;
    bool apnea = false;

    /* 1. Prime until a peak is registered and timer resets */
    for (int i = 0; i < 100; i++) {
        float t = (float)i * 0.05f;
        float s = 1.65f + 0.45f * sinf(2.0f * (float)M_PI * 0.25f * t);
        float f = dsp_filter_step(s);
        bool peak = dsp_peak_detect_step(f, false, &bpm, &apnea);
        if (peak) break;
    }
    TEST_ASSERT(!apnea, "Apnea is FALSE at breath peak");

    /* 2. Cease breathing for 7.0 seconds (140 samples < 160 threshold) */
    for (int i = 0; i < 140; i++) {
        float f = dsp_filter_step(1.65f); // Flatline
        dsp_peak_detect_step(f, false, &bpm, &apnea);
    }
    TEST_ASSERT(!apnea, "Apnea is NOT triggered at 7.0 seconds (under 8.0s threshold)");

    /* 3. Continue cessation to 8.5 seconds (additional 30 samples = 170 total >= 160 threshold) */
    for (int i = 0; i < 30; i++) {
        float f = dsp_filter_step(1.65f);
        dsp_peak_detect_step(f, false, &bpm, &apnea);
    }
    TEST_ASSERT(apnea, "Apnea flag triggers TRUE strictly after >= 8.0 seconds of cessation");
}

/* =========================================================================
 * Test 5: Steering Inactivity Deadband Trigger
 * ========================================================================= */
static void test_steering_inactivity(void)
{
    printf("\n--- Test 5: Steering Inactivity Detection (7.0s Deadband) ---\n");
    dsp_pipeline_reset();

    float angle = 0.0f;
    bool inactive = false;

    /* 1. Active driving for 7 seconds (continuous oscillations) */
    for (int i = 0; i < 140; i++) {
        float t = (float)i * 0.05f;
        float steer_v = 1.65f + 0.30f * sinf(0.5f * t);
        inactive = dsp_steering_inactivity_step(steer_v, &angle);
    }
    TEST_ASSERT(!inactive, "Steering detected as ACTIVE during wheel movement");

    /* 2. Frozen/Inactive steering for 7 seconds (140 samples constant 1.65V) */
    for (int i = 0; i < 140; i++) {
        inactive = dsp_steering_inactivity_step(1.65f, &angle);
    }
    TEST_ASSERT(inactive, "Steering inactivity flag triggers TRUE after 7.0s of static hold");

    /* 3. Driver twitches steering wheel (deflection +0.5V) -> Must recover! */
    inactive = dsp_steering_inactivity_step(2.15f, &angle);
    /* Notice: the buffer now has 2.15 - 1.65 = 0.50V > deadband (0.04V) */
    TEST_ASSERT(!inactive, "Steering immediately returns to ACTIVE upon driver wheel jerk");
}

int main(void)
{
    printf("===========================================================\n");
    printf("   DRIVER DROWSINESS DSP PIPELINE HOST UNIT TEST SUITE     \n");
    printf("===========================================================\n");

    test_filter_noise_attenuation();
    test_artifact_rejection();
    test_bpm_calculation();
    test_apnea_detection();
    test_steering_inactivity();

    printf("\n===========================================================\n");
    printf(" Test Summary: %d Passed, %d Failed\n", s_tests_passed, s_tests_failed);
    printf("===========================================================\n");

    return (s_tests_failed == 0) ? 0 : 1;
}
