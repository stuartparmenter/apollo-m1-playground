// bios_boot_helpers.h
#pragma once
#include "esp_app_desc.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_mac.h"
#include "esp_idf_version.h"
#include "lvgl.h"

static inline std::string bios_get_version() {
  const esp_app_desc_t* app_desc = esp_app_get_description();
  char buf[64];
  snprintf(buf, sizeof(buf), "VER:%s", app_desc->version);
  return std::string(buf);
}


static inline std::string bios_cpu_string() {
  esp_chip_info_t ci;
  esp_chip_info(&ci);
  const char *model = "ESP32";
  switch (ci.model) {
    case CHIP_ESP32:   model = "ESP32";    break;
    case CHIP_ESP32S2: model = "ESP32-S2"; break;
    case CHIP_ESP32S3: model = "ESP32-S3"; break;
    case CHIP_ESP32C3: model = "ESP32-C3"; break;
    case CHIP_ESP32C6: model = "ESP32-C6"; break;
    case CHIP_ESP32H2: model = "ESP32-H2"; break;
    default:           model = "ESP32";    break;
  }
  char buf[32];
  snprintf(buf, sizeof(buf), "CPU:%s", model);
  return std::string(buf);
}

static inline std::string bios_mem_string() {
  // No direct PSRAM calls — link-safe on all targets
  size_t ram_total   = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
  size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);  // 0 if no PSRAM

  char buf[32];
  if (psram_total > 0) {
    snprintf(buf, sizeof(buf), "MEM:%luk+%luk",
             (unsigned long)(ram_total / 1024),
             (unsigned long)(psram_total / 1024));
  } else {
    snprintf(buf, sizeof(buf), "MEM:%luk", (unsigned long)(ram_total / 1024));
  }
  return std::string(buf);
}

static inline std::string bios_id_string() {
  uint8_t mac[6] = {0};
  // Portable across IDF versions
  esp_read_mac(mac, ESP_MAC_WIFI_STA);

  char buf[32];
  snprintf(buf, sizeof(buf), "ID:%02X%02X%02X", mac[3], mac[4], mac[5]);
  return std::string(buf);
}

static inline std::string bios_screen_size() {
#if LVGL_VERSION_MAJOR >= 9
  lv_obj_t *scr = lv_screen_active();
  lv_display_t *disp = lv_obj_get_display(scr);
  if (!disp) return "WxD:unknown";
  int w = lv_display_get_horizontal_resolution(disp);
  int h = lv_display_get_vertical_resolution(disp);
#else
  lv_obj_t *scr = lv_scr_act();
  lv_disp_t *disp = lv_obj_get_disp(scr);
  if (!disp) return "WxD:unknown";
  int w = lv_disp_get_hor_res(disp);
  int h = lv_disp_get_ver_res(disp);
#endif

  char buf[32];
  std::snprintf(buf, sizeof(buf), "WxD:%dx%d", w, h);
  return std::string(buf);
}