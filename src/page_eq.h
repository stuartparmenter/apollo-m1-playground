// © 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once

// -------- Debug/metrics toggle --------
// Enable metrics + logs with: -DEQ_FFT_METRICS=1
#ifndef EQ_FFT_METRICS
#define EQ_FFT_METRICS 0
#endif

#include <vector>
#include <algorithm>
#include <math.h>
#include <stdint.h>
#include <limits.h>   // INT32_MAX / INT32_MIN

#include "esp_dsp.h"
#include "esp_heap_caps.h"
#include "esphome/core/log.h"
#include "esp_timer.h"
#include <lvgl.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace EqFftRenderer {

// -------- Config (change via setup) --------
static int   g_fft_n      = 512;        // 256/512/1024
static int   g_num_bars   = 16;         // explicit bar count (never auto-changed)
static float g_fs         = 16000.0f;   // must match YAML sample_rate
static float g_peak_decay = 0.02f;      // slower, nicer peaks
static float g_smooth     = 0.25f;      // EMA amount for bars
static float g_gain_db    = 6.0f;       // visualization gain
static float g_noise_db   = -50.0f;     // visualization floor reference (dB)
static float g_fmin       = 250.0f;     // min frequency for bars (Hz)
static float g_fmax_frac  = 0.45f;      // fraction of fs used as fmax

// Silence gating: when time-domain RMS is very low, decay bars to 0
static float g_noise_gate_db = -62.0f;  // gate threshold on last_rms_db
static float g_idle_decay    = 0.04f;   // how fast bars fall when gated

// Low-shelf bias for first buckets (de-emphasize LF in quiet rooms)
static float g_bar0_scale = 0.50f;      // bar 0 multiplier
static float g_bar1_scale = 0.70f;      // bar 1 multiplier

// -------- State --------
static bool   g_inited = false;
static float *g_fft_w  = nullptr;      // esp-dsp twiddle table (only)

// time/spectral buffers
static std::vector<float> g_ring;
static size_t g_rpos = 0;

static std::vector<float> g_window;
static std::vector<float> g_fft_in;

static std::vector<float> g_bar;       // [0..1]
static std::vector<float> g_peak;      // [0..1]

// Thread-safe copies for rendering
static std::vector<float> g_bar_copy;
static std::vector<float> g_peak_copy;
static SemaphoreHandle_t g_data_mutex = nullptr;
static bool g_data_ready = false;

struct BarPair { lv_obj_t *bar; lv_obj_t *peak; };
static std::vector<BarPair> g_rects;

// DC blocker (one-pole HPF)
static float g_hp_prev_x = 0.0f, g_hp_prev_y = 0.0f;
static const float HP_A = 0.990f;

// (kept for parity if you wire this elsewhere)
static bool g_format_detected = false;
static bool g_is_24bit = true;
static int  g_format_check_count = 0;

// Always keep last RMS/max for gating (even if metrics disabled)
static float   g_last_rms     = 0.0f;     // linear RMS of time domain (pre-FFT)
static float   g_last_rms_db  = -120.0f;  // dBFS-ish (0 dB ~ full scale)
static float   g_last_max_abs = 0.0f;     // max |sample| of recent frame

static const char *TAG = "eq_fft";
#if EQ_FFT_METRICS
static int64_t g_last_log_ms  = 0;
#endif

inline float last_rms_db()  { return g_last_rms_db; }
inline float last_max_abs() { return g_last_max_abs; }

// -------- Optional helper (available for your external use) --------
inline int suggest_bar_count(lv_obj_t *parent,
                             int bar_px = 7,
                             int gap_px = 1,
                             int min_bars = 8,
                             int max_bars = 96,
                             bool round_to_multiple_of_8 = true) {
  int width = lv_obj_get_width(parent);
  //ESP_LOGI(TAG, "suggest_bar_count: parent width = %d", width);

  if (width <= 0) {
    lv_disp_t *disp = lv_disp_get_default();
    if (disp) {
      width = lv_disp_get_hor_res(disp);
      //ESP_LOGI(TAG, "Using display width instead: %d", width);
    }
  }

  int candidate = (width + gap_px) / (bar_px + gap_px);
  //ESP_LOGI(TAG, "Calculated candidate bars: %d", candidate);

  if (round_to_multiple_of_8) {
    candidate = (candidate + 7) & ~7;
    //ESP_LOGI(TAG, "After rounding to multiple of 8: %d", candidate);
  }

  candidate = std::max(min_bars, std::min(max_bars, candidate));
  //ESP_LOGI(TAG, "Final bar count: %d", candidate);
  return candidate;
}

// -------- Internal helpers --------
static inline void free_tables_() {
  if (g_fft_w) { heap_caps_free(g_fft_w); g_fft_w = nullptr; }
}

static inline void ensure_init_() {
  if (g_inited) return;

  // Allocate esp-dsp table (twiddles).
  g_fft_w = (float*) heap_caps_malloc(g_fft_n * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!g_fft_w) {
    // fallback to PSRAM (slower but safe)
    g_fft_w = (float*) heap_caps_malloc(g_fft_n * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  if (!g_fft_w) return;

  if (dsps_fft2r_init_fc32(g_fft_w, g_fft_n) != ESP_OK) {
    free_tables_();
    return;
  }

  // Buffers
  g_ring.assign(g_fft_n, 0.0f);
  g_rpos = 0;

  // Hann window
  g_window.resize(g_fft_n);
  for (int n = 0; n < g_fft_n; n++) {
    g_window[n] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * n / (g_fft_n - 1)));
  }

  g_fft_in.assign(g_fft_n * 2, 0.0f);  // interleaved complex [re, im]

  g_bar.assign(g_num_bars, 0.0f);
  g_peak.assign(g_num_bars, 0.0f);
  g_bar_copy.assign(g_num_bars, 0.0f);
  g_peak_copy.assign(g_num_bars, 0.0f);

  g_hp_prev_x = g_hp_prev_y = 0.0f;
  g_format_detected = false;
  g_format_check_count = 0;

  if (g_data_mutex == nullptr) {
    g_data_mutex = xSemaphoreCreateMutex();
  }

  g_inited = true;
}

static inline void ensure_lv_rects_(lv_obj_t *parent) {
  if ((int)g_rects.size() == g_num_bars) return;
  g_rects.clear();
  g_rects.resize(g_num_bars);
  for (int i = 0; i < g_num_bars; i++) {
    g_rects[i].bar  = lv_obj_create(parent);
    g_rects[i].peak = lv_obj_create(parent);
    lv_obj_remove_style_all(g_rects[i].bar);
    lv_obj_remove_style_all(g_rects[i].peak);
    lv_obj_set_style_bg_color(g_rects[i].bar,  lv_color_hex(0x00FFD0), LV_PART_MAIN);
    lv_obj_set_style_bg_color(g_rects[i].peak, lv_color_hex(0xFF4000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa  (g_rects[i].bar,  LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_opa  (g_rects[i].peak, LV_OPA_COVER, LV_PART_MAIN);
  }
}

// -------- Lifecycle --------
inline void teardown() {
  g_inited = false;
  g_rects.clear();
  g_ring.clear();
  g_window.clear();
  g_fft_in.clear();
  g_bar.clear();
  g_peak.clear();
  g_bar_copy.clear();
  g_peak_copy.clear();
  free_tables_();
}

inline void setup(int fft_n = 512,
                  int num_bars = 16,
                  float fs = 16000.0f,
                  float peak_decay = 0.02f,
                  float smooth = 0.25f,
                  float gain_db = 6.0f,
                  float noise_db = -50.0f,
                  float fmin_hz = 250.0f,
                  float fmax_frac = 0.45f) {
  ESP_LOGI(TAG, "setup() called with fft_n=%d, bars=%d", fft_n, num_bars);

  g_fft_n = fft_n;
  g_num_bars = num_bars;
  g_fs = fs;
  g_peak_decay = peak_decay;
  g_smooth = smooth;
  g_gain_db = gain_db;
  g_noise_db = noise_db;
  g_fmin = fmin_hz;
  g_fmax_frac = fmax_frac;
  teardown();
  ensure_init_();
}

inline void update_bar_count(int new_bars) {
  if (new_bars == g_num_bars) return;

  ESP_LOGI(TAG, "Updating bar count from %d to %d", g_num_bars, new_bars);
  g_num_bars = new_bars;

  // Just resize the bar arrays, don't touch FFT
  g_bar.assign(g_num_bars, 0.0f);
  g_peak.assign(g_num_bars, 0.0f);
  g_bar_copy.assign(g_num_bars, 0.0f);
  g_peak_copy.assign(g_num_bars, 0.0f);
  g_rects.clear();  // Force recreation on next render
}

// -------- Audio ingest + analysis --------
// Process audio in mic thread - NO LVGL calls here
// buf.size() must be a multiple of 4 (24-bit left-justified in 32-bit words)
inline void process_audio(const std::vector<uint8_t> &buf) {
  ensure_init_();
  if (!g_inited || buf.empty()) return;

  const int32_t *raw = (const int32_t*) buf.data();
  const int nsamp = (int)(buf.size() / 4);
  if (nsamp <= 0) return;

#if EQ_FFT_METRICS
  // Debug logging (raw min/max/avg) — compiled only if metrics enabled
  static int debug_counter = 0;
  if (debug_counter++ % 100 == 0) {
    int32_t min_val = INT32_MAX;
    int32_t max_val = INT32_MIN;
    int64_t sum = 0;
    for (int i = 0; i < nsamp; i++) {
      if (raw[i] < min_val) min_val = raw[i];
      if (raw[i] > max_val) max_val = raw[i];
      sum += raw[i];
    }
    int32_t avg = (nsamp > 0) ? (int32_t)(sum / nsamp) : 0;
    ESP_LOGD(TAG, "Raw data range: min=%d (0x%08X), max=%d (0x%08X), avg=%d",
             min_val, (unsigned)min_val, max_val, (unsigned)max_val, avg);
  }
#endif

  // Accumulators for RMS / max (always active — used for gating)
  double acc2 = 0.0;
  float  max_abs = 0.0f;

  // Convert 24-bit left-justified -> [-1,1] float, DC block
  for (int i = 0; i < nsamp; i++) {
    // raw 32-bit word contains 24-bit sample left-justified
    float s = (float)(raw[i] / 256) * (1.0f / 8388608.0f);  // 24-bit full-scale
    // DC high-pass (one-pole)
    float y = s - g_hp_prev_x + HP_A * g_hp_prev_y;
    g_hp_prev_x = s;
    g_hp_prev_y = y;
    s = y;

    acc2 += (double)s * (double)s;
    float a = fabsf(s);
    if (a > max_abs) max_abs = a;
  }

  g_last_rms = sqrt(acc2 / (double)nsamp);
  g_last_rms_db = 20.0f * log10f(g_last_rms + 1e-12f);
  g_last_max_abs = max_abs;

  // Ensure buffers
  if ((int)g_ring.size() != g_fft_n) {
    g_ring.assign(g_fft_n, 0.0f);
    g_rpos = 0;
  }
  if ((int)g_fft_in.size() != g_fft_n * 2) g_fft_in.assign(g_fft_n * 2, 0.0f);

  // Append new samples into ring (use latest HPF output)
  for (int i = 0; i < nsamp; i++) {
    g_ring[g_rpos] = g_hp_prev_y;
    g_rpos = (g_rpos + 1) % g_fft_n;
  }

  // Copy most-recent frame to FFT buffer with window; imag=0
  for (int n = 0; n < g_fft_n; n++) {
    int idx = (int)((g_rpos + n) % g_fft_n);
    float w = (g_window.empty() ? 1.0f : g_window[n]);
    g_fft_in[2*n + 0] = g_ring[idx] * w; // real
    g_fft_in[2*n + 1] = 0.0f;            // imag
  }

  // FFT
  dsps_fft2r_fc32(g_fft_in.data(), g_fft_n);
  dsps_bit_rev_fc32(g_fft_in.data(), g_fft_n);
  dsps_cplx2reC_fc32(g_fft_in.data(), g_fft_n);

  // Convert spectrum -> bars
  if ((int)g_bar.size() != g_num_bars) g_bar.assign(g_num_bars, 0.0f);
  if ((int)g_peak.size() != g_num_bars) g_peak.assign(g_num_bars, 0.0f);

  const int n_bins = g_fft_n / 2;
  const float fmax = std::min(7500.0f, g_fmax_frac * g_fs);

  auto bin_of_hz = [&](float f){
    float b = f * g_fft_n / g_fs;
    if (b < 1.0f) b = 1.0f;                 // skip DC bin
    if (b > (float)(n_bins - 1)) b = (float)(n_bins - 1);
    return (int)b;
  };

  if (g_last_rms_db < g_noise_gate_db) {
    // Silence gating: decay bars towards 0
    for (int i = 0; i < g_num_bars; i++) {
      g_bar[i]  = std::max(0.0f, g_bar[i]  - g_idle_decay);
      g_peak[i] = std::max(0.0f, g_peak[i] - g_idle_decay);
    }
  } else {
    for (int i = 0; i < g_num_bars; i++) {
      float t0 = (float)i / (float)g_num_bars;
      float t1 = (float)(i + 1) / (float)g_num_bars;
      float f0 = g_fmin * powf(fmax / g_fmin, t0);
      float f1 = g_fmin * powf(fmax / g_fmin, t1);

      int b0 = bin_of_hz(f0);
      int b1 = bin_of_hz(f1);
      if (b1 <= b0) b1 = b0 + 1;

      float acc = 0.0f;
      for (int b = b0; b < b1; b++) {
        float re = g_fft_in[2*b + 0];
        float im = g_fft_in[2*b + 1];
        acc += re*re + im*im;
      }
      acc /= (float)(b1 - b0);

      float db = 10.0f * log10f(acc + 1e-12f) + g_gain_db;
      float norm = (db - g_noise_db) / (0.0f - g_noise_db);

#if EQ_FFT_METRICS
      // occasional per-bar diagnostics (very limited)
      if (i == 0 && (esp_timer_get_time() / 1000) % 1000 < 100) {
        ESP_LOGD(TAG, "Bar[0]: acc=%e, db=%.1f, norm=%.3f (gain=%.0f, floor=%.0f)",
                 acc, db, norm, g_gain_db, g_noise_db);
      }
#endif

      // Soft-knee near the floor
      if (norm < 0.12f) {
        float t = norm / 0.12f;
        norm = 0.12f * (t * t);
      }

      // Optional LF de-emphasis
      if (i == 0) norm *= g_bar0_scale;
      if (i == 1) norm *= g_bar1_scale;

      norm = std::clamp(norm, 0.0f, 1.0f);

      // EMA smoothing
      g_bar[i] = g_bar[i] * (1.0f - g_smooth) + norm * g_smooth;

      // Peak decay
      if (g_peak[i] < g_bar[i]) g_peak[i] = g_bar[i];
      else g_peak[i] = std::max(0.0f, g_peak[i] - g_peak_decay);
    }
  }

  // Publish copies for render
  if (g_data_mutex != nullptr && xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    g_bar_copy = g_bar;
    g_peak_copy = g_peak;
    g_data_ready = true;
    xSemaphoreGive(g_data_mutex);
  }

#if EQ_FFT_METRICS
  // Rate-limited debug summary (every ~500 ms)
  int64_t now_ms = esp_timer_get_time() / 1000;
  if (now_ms - g_last_log_ms >= 500) {
    g_last_log_ms = now_ms;
    float b0 = (g_bar.empty() ? 0.0f : g_bar[0]);
    float bN = (g_bar.empty() ? 0.0f : g_bar.back());
    ESP_LOGD(TAG, "mic: rms=%.1f dB, max=%.3f, bars[0]=%.2f bars[last]=%.2f (nsamp=%d)",
             g_last_rms_db, g_last_max_abs, b0, bN, nsamp);
  }
#endif
}

// -------- Rendering (main thread) --------
inline void render_bars(lv_obj_t *parent) {
  if (!g_inited) return;

  int W = lv_obj_get_width(parent);
  int H = lv_obj_get_height(parent);
  if (W <= 0 || H <= 0) return;

  // Never change g_num_bars here — respect what setup() was given.
  if ((int)g_rects.size() != g_num_bars) {
    ensure_lv_rects_(parent);
  }

  // Take snapshot
  std::vector<float> bar_local, peak_local;
  bool ready = false;
  if (g_data_mutex != nullptr && xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    if (g_data_ready) {
      bar_local = g_bar_copy;
      peak_local = g_peak_copy;
      ready = true;
    }
    xSemaphoreGive(g_data_mutex);
  }
  if (!ready) return;

  // Draw bars
  const int gap = 1;
  int barW = (W - gap * (g_num_bars - 1)) / g_num_bars;
  if (barW < 1) barW = 1;

  for (int i = 0; i < g_num_bars; i++) {
    int x0 = i * (barW + gap);
    int h  = (int)lrintf(bar_local[i] * H);
    if (h < 1) h = 1;
    int y0 = H - h;
    int y_peak = H - (int)lrintf(peak_local[i] * H) - 1;
    if (y_peak < 0) y_peak = 0;

    if (i >= (int)g_rects.size()) break; // safety
    lv_obj_set_size(g_rects[i].bar,  barW, h);
    lv_obj_set_pos (g_rects[i].bar,  x0,   y0);

    lv_obj_set_size(g_rects[i].peak, barW, 1);
    lv_obj_set_pos (g_rects[i].peak, x0,   y_peak);
  }
}

} // namespace EqFftRenderer
