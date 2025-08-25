// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// Fireworks + Chipmunk2D on LVGL canvas
// - Single-task Chipmunk stepping (LVGL timers only) → no concurrency
// - Bodies-only integration (no shapes) → no BBTree crashes
// - Burst patterns: peony/chrysanthemum, ring, star(5/6), palm, willow, crossette
// - Culling only when particles actually leave the screen (with a hard TTL)
// - Two APIs:
//     * Page-layer: fireworks_physics_on_load(layer) / _on_unload(layer) → auto canvas
//     * Canvas-first: fireworks_physics_attach_to_canvas(canvas) / _detach_from_canvas(canvas)

#pragma once

extern "C" {
#include <lvgl.h>
#if !defined(LV_USE_CANVAS) || (LV_USE_CANVAS == 0)
#  error "LVGL canvas is disabled; enable LV_USE_CANVAS in your lv_conf.h."
#endif
}

#include <math.h>
#include <stdint.h>
#include <vector>
#include <algorithm>
#include <esp_random.h>  // esp_random()

#ifndef INFINITY
#define INFINITY (1.0f/0.0f)
#endif

// Chipmunk headers (floats on ESP32)
#ifndef CP_USE_DOUBLES
#define CP_USE_DOUBLES 0
#endif
#include <chipmunk/chipmunk.h>

namespace fireworks_canvas_detail {

// ---------- tunables ----------
static constexpr uint8_t  TRAIL_FADE_OPA     = 32;    // lower = longer trails
static constexpr float    GRAVITY_PX_S2      = 180.0f;
static constexpr uint32_t TICK_MS            = 33;    // ~30 FPS
static constexpr uint32_t SPAWN_MS           = 40;

static constexpr int      CULL_MARGIN        = 2;     // allow slight overscan
static constexpr uint32_t HARD_TTL_MS        = 3200;  // absolute max life
static constexpr uint32_t SOFT_TTL_PAD_MS    = 800;   // extra beyond life_ms

// ---------- helpers ----------
static inline uint32_t now_ms() { return lv_tick_get(); }

static inline bool obj_alive_(lv_obj_t* o) {
  if (!o) return false;
  if (!lv_obj_is_valid(o)) return false;
#ifdef LV_OBJ_FLAG_DELETE_PENDING
  if (lv_obj_has_flag(o, LV_OBJ_FLAG_DELETE_PENDING)) return false;
#endif
  return true;
}

static inline lv_color_t pick_warm() {
  static const lv_color_t WARM[] = {
    lv_color_hex(0xFFFFFF), lv_color_hex(0xFFF4CC), lv_color_hex(0xFFD27F),
    lv_color_hex(0xFFB84D), lv_color_hex(0xFF9E2D)
  };
  return WARM[esp_random() % (sizeof(WARM)/sizeof(WARM[0]))];
}
static inline lv_color_t pick_accent() {
  static const lv_color_t ACCENT[] = { lv_color_hex(0x00E6E6), lv_color_hex(0x8A2BE2) };
  return ACCENT[esp_random() % (sizeof(ACCENT)/sizeof(ACCENT[0]))];
}
static inline lv_color_t tweak(lv_color_t c) {
  uint8_t w = 16 + (esp_random() % 48);
  return lv_color_mix(lv_color_white(), c, w);
}

// ---------- data ----------
struct BodyRef {
  cpBody*   body{nullptr};
  bool      alive{true};
  bool      is_rocket{false};

  lv_color_t color{lv_color_white()};
  uint8_t   px{1};
  uint32_t  born_ms{0};
  uint32_t  fuse_ms{0};
  uint32_t  life_ms{1000};

  // crossette
  bool      can_split{false};
  bool      split_done{false};
  uint32_t  split_ms{0};
};

struct Ctx {
  // UI
  lv_obj_t* layer{nullptr};
  lv_obj_t* canvas{nullptr};
  uint8_t*  buf{nullptr};
  uint16_t  W{64}, H{64};

  // timers
  lv_timer_t* tick{nullptr};
  lv_timer_t* spawner{nullptr};
  bool alive{true};

  // physics (page-owned)
  cpSpace* space{nullptr};
  bool     stepping{false};
  float    step_dt{1.0f/60.0f};

  // objects
  std::vector<BodyRef*> items;

  // spawn control
  uint8_t  inflight{0};
  uint8_t  inflight_max{4};
  uint32_t spawn_last_ms{0};

  // visuals
  uint8_t  fade_opa{TRAIL_FADE_OPA};
};

// ---------- drawing ----------
static inline void draw_px_rect_(lv_obj_t* canvas, int x, int y, uint8_t px, lv_color_t color) {
  if (!obj_alive_(canvas)) return;
  int W = lv_obj_get_width(canvas), H = lv_obj_get_height(canvas);
  if (x >= W || y >= H || x + (int)px <= 0 || y + (int)px <= 0) return;

  int sx = std::max(0, x), sy = std::max(0, y);
  int ex = std::min(W, x + (int)px), ey = std::min(H, y + (int)px);

  lv_draw_rect_dsc_t d; lv_draw_rect_dsc_init(&d);
  d.bg_color = color; d.bg_opa = LV_OPA_COVER; d.border_opa = LV_OPA_TRANSP; d.radius = 0;
  lv_canvas_draw_rect(canvas, sx, sy, ex - sx, ey - sy, &d);
}

// ---------- memory ----------
static inline void free_bodyref_(Ctx* c, BodyRef* br) {
  if (!br) return;
  br->alive = false;
  if (c && c->space && br->body) {
    cpSpaceRemoveBody(c->space, br->body);
    cpBodyFree(br->body);
    br->body = nullptr;
  }
  delete br;
}
static inline void clear_items_(Ctx* c) {
  for (auto* it : c->items) free_bodyref_(c, it);
  c->items.clear();
}

// ---------- spawners (bodies only; no shapes) ----------
static BodyRef* spawn_particle_(Ctx* c, int x, int y, float vx, float vy,
                                lv_color_t base, uint8_t px=1, uint32_t life_ms=1000,
                                bool can_split=false, uint32_t split_ms=0) {
  if (!c || !c->alive || !c->space || c->stepping) return nullptr;
  BodyRef* br = new BodyRef();
  br->is_rocket = false;
  br->born_ms   = now_ms();
  br->life_ms   = life_ms;
  br->px        = px;
  br->color     = tweak(base);
  br->can_split = can_split;
  br->split_ms  = split_ms;

  const cpFloat m = 1.0f;
  br->body = cpBodyNew(m, INFINITY); // no rotation needed
  cpBodySetPosition(br->body, cpv((cpFloat)x, (cpFloat)y));
  cpBodySetVelocity(br->body, cpv(vx, vy));
  cpSpaceAddBody(c->space, br->body);

  c->items.push_back(br);
  return br;
}

static BodyRef* spawn_rocket_(Ctx* c, int x) {
  if (!c || !c->alive || !c->space || c->stepping) return nullptr;
  BodyRef* br = new BodyRef();
  br->is_rocket = true;
  br->born_ms   = now_ms();
  br->fuse_ms   = 260 + (esp_random()%220);  // earlier → mid-screen bursts
  br->px        = 1;
  br->color     = lv_color_hex(0xFFFFFF);

  float vx = (float)((int)(esp_random()%31) - 15) * 0.12f;
  float vy = - (120.0f + (esp_random()%40));

  const cpFloat m = 0.8f;
  br->body = cpBodyNew(m, INFINITY);
  cpBodySetPosition(br->body, cpv((cpFloat)x, (cpFloat)(c->H - 1)));
  cpBodySetVelocity(br->body, cpv(vx, vy));
  cpSpaceAddBody(c->space, br->body);

  c->items.push_back(br);
  c->inflight++;
  return br;
}

// ---------- patterns ----------
enum class BurstKind : uint8_t { PEONY, CHRYS, RING, STAR5, STAR6, PALM, WILLOW, CROSSETTE };

static BurstKind pick_burst_kind() {
  uint8_t r = esp_random() % 100;
  if (r < 28) return BurstKind::PEONY;
  if (r < 45) return BurstKind::RING;
  if (r < 58) return BurstKind::STAR5;
  if (r < 68) return BurstKind::PALM;
  if (r < 79) return BurstKind::WILLOW;
  if (r < 88) return BurstKind::CHRYS;
  if (r < 96) return BurstKind::STAR6;
  return BurstKind::CROSSETTE;
}

static void burst_peony_(Ctx* c, int cx, int cy, bool chrys=false) {
  bool accent = (esp_random() % 6 == 0);
  lv_color_t base = accent ? pick_accent() : pick_warm();
  int n      = 20 + (esp_random() % 18);
  float s0   = 70.0f + (esp_random()%50);
  int life   = chrys ? (1100 + (esp_random()%600)) : (800 + (esp_random()%400));
  for (int i=0;i<n;i++) {
    float a = (2.0f * (float)M_PI * i) / n;
    float jitter = ((int)(esp_random()%101) - 50) * 0.012f;
    float sp = s0 * (0.90f + (esp_random()%21)/100.0f);
    float vx = cosf(a + jitter) * sp;
    float vy = sinf(a + jitter) * sp;
    uint8_t px = (esp_random()%4==0) ? 2 : 1;
    spawn_particle_(c, cx, cy, vx, vy, base, px, life);
  }
}
static void burst_ring_(Ctx* c, int cx, int cy) {
  bool accent = (esp_random() % 3 == 0);
  lv_color_t base = accent ? pick_accent() : pick_warm();
  int n    = 26 + (esp_random()%16);
  float s  = 85.0f + (esp_random()%35);
  int life = 850 + (esp_random()%450);
  for (int i=0;i<n;i++) {
    float a = (2.0f * (float)M_PI * i) / n;
    float vx = cosf(a) * s, vy = sinf(a) * s;
    spawn_particle_(c, cx, cy, vx, vy, base, 1, life);
  }
}
static void burst_starN_(Ctx* c, int cx, int cy, int N) {
  lv_color_t base = pick_warm();
  float s  = 95.0f + (esp_random()%35);
  int life = 900 + (esp_random()%400);
  for (int i=0;i<N;i++) {
    float a = (2.0f * (float)M_PI * i) / N;
    int rays = 3 + (esp_random()%2);
    for (int k=0;k<rays;k++) {
      float t = 0.55f + 0.18f * k; // 55%, 73%, 91%
      float vx = cosf(a) * s * t, vy = sinf(a) * s * t;
      spawn_particle_(c, cx, cy, vx, vy, base, 1, life);
    }
  }
  burst_ring_(c, cx, cy);
}
static void burst_palm_(Ctx* c, int cx, int cy) {
  lv_color_t base = pick_warm();
  int fronds = 8 + (esp_random()%6);
  int life   = 1000 + (esp_random()%500);
  for (int i=0;i<fronds;i++) {
    float spread = (35.0f * (float)M_PI/180.0f);
    float a = -((float)M_PI/2) + (-spread + 2*spread*((esp_random()%100)/100.0f));
    float s = 105.0f + (esp_random()%25);
    float vx = cosf(a) * s, vy = sinf(a) * s;
    spawn_particle_(c, cx, cy, vx, vy, base, 2, life);
  }
}
static void burst_willow_(Ctx* c, int cx, int cy) {
  lv_color_t base = pick_warm();
  int n    = 18 + (esp_random()%10);
  float s  = 55.0f + (esp_random()%15);
  int life = 1600 + (esp_random()%800);
  for (int i=0;i<n;i++) {
    float a = (2.0f * (float)M_PI * i) / n;
    float vx = cosf(a) * s, vy = sinf(a) * s + 2.0f; // gentle droop
    spawn_particle_(c, cx, cy, vx, vy, base, 1, life);
  }
}
static void burst_crossette_(Ctx* c, int cx, int cy) {
  lv_color_t base = pick_warm();
  int n    = 10 + (esp_random()%6);
  float s  = 80.0f + (esp_random()%20);
  int life = 1100 + (esp_random()%300);
  for (int i=0;i<n;i++) {
    float a = (2.0f * (float)M_PI * i) / n;
    float vx = cosf(a) * s, vy = sinf(a) * s;
    uint32_t split_ms = 240 + (esp_random()%220);
    spawn_particle_(c, cx, cy, vx, vy, base, 1, life, /*can_split=*/true, split_ms);
  }
}
static void burst_(Ctx* c, int cx, int cy, BurstKind kind) {
  switch (kind) {
    case BurstKind::PEONY:     burst_peony_(c, cx, cy, false); break;
    case BurstKind::CHRYS:     burst_peony_(c, cx, cy, true);  break;
    case BurstKind::RING:      burst_ring_(c, cx, cy);         break;
    case BurstKind::STAR5:     burst_starN_(c, cx, cy, 5);     break;
    case BurstKind::STAR6:     burst_starN_(c, cx, cy, 6);     break;
    case BurstKind::PALM:      burst_palm_(c, cx, cy);         break;
    case BurstKind::WILLOW:    burst_willow_(c, cx, cy);       break;
    case BurstKind::CROSSETTE: burst_crossette_(c, cx, cy);    break;
  }
}
static void maybe_split_crossettes_(Ctx* c, uint32_t tnow) {
  if (!c || !c->space || c->stepping) return;
  std::vector<BodyRef*> to_split;
  for (auto* it : c->items) {
    if (!it || it->is_rocket || !it->alive || it->split_done || !it->can_split) continue;
    if (tnow - it->born_ms >= it->split_ms) to_split.push_back(it);
  }
  for (auto* it : to_split) {
    cpVect p = it->body ? cpBodyGetPosition(it->body) : cpv(0,0);
    float sv = 70.0f + (esp_random()%20);
    spawn_particle_(c, (int)p.x, (int)p.y,  sv,  0, it->color, 1, 600);
    spawn_particle_(c, (int)p.x, (int)p.y, -sv,  0, it->color, 1, 600);
    spawn_particle_(c, (int)p.x, (int)p.y,  0,  sv, it->color, 1, 600);
    spawn_particle_(c, (int)p.x, (int)p.y,  0, -sv, it->color, 1, 600);
    it->split_done = true;
    it->alive = false;
  }
}

// ---------- render + step ----------
static void render_(Ctx* c) {
  if (!c || !c->alive) return;

  // trails / motion blur
  if (obj_alive_(c->canvas)) {
    lv_canvas_fill_bg(c->canvas, lv_color_black(), c->fade_opa);
  }

  // draw bodies
  for (auto* it : c->items) {
    if (!it || !it->alive || !it->body) continue;
    cpVect p = cpBodyGetPosition(it->body);
    draw_px_rect_(c->canvas, (int)p.x, (int)p.y, it->px, it->color);
  }

  const uint32_t tnow = now_ms();

  // crossette splits
  maybe_split_crossettes_(c, tnow);

  // burst rockets that reached fuse (no clamping — burst where it is)
  std::vector<BodyRef*> to_burst;
  for (auto* it : c->items) {
    if (!it || !it->alive || !it->is_rocket || !it->body) continue;
    if (tnow - it->born_ms >= it->fuse_ms) to_burst.push_back(it);
  }
  for (auto* it : to_burst) {
    cpVect p = cpBodyGetPosition(it->body);
    burst_(c, (int)p.x, (int)p.y, pick_burst_kind());
    it->alive = false; // retire rocket; GC below will remove
    if (c->inflight) c->inflight--;
  }

  // lifetime/visibility GC — keep until actually off-screen (with hard TTL)
  std::vector<BodyRef*> keep;
  keep.reserve(c->items.size());

  for (auto* it : c->items) {
    if (!it) continue;

    cpVect p = it->body ? cpBodyGetPosition(it->body) : cpv(-9999, -9999);
    bool off = (p.x < -CULL_MARGIN || p.x > (c->W - 1 + CULL_MARGIN) ||
                p.y < -CULL_MARGIN || p.y > (c->H - 1 + CULL_MARGIN));
    uint32_t age = now_ms() - it->born_ms;
    bool hard_expired = age > std::max<uint32_t>(it->life_ms + SOFT_TTL_PAD_MS, HARD_TTL_MS);

    // rockets: if retired or off, free; else keep
    if (it->is_rocket) {
      if (!it->alive || off) { free_bodyref_(c, it); }
      else { keep.push_back(it); }
      continue;
    }

    // particles: keep until off-screen (or hard TTL)
    if (!it->alive || off || hard_expired) {
      free_bodyref_(c, it);
    } else {
      keep.push_back(it);
    }
  }
  c->items.swap(keep);
}

static void step_space_(Ctx* c) {
  if (!c || !c->space) return;
  c->stepping = true;
  cpSpaceStep(c->space, c->step_dt * 0.5f);
  cpSpaceStep(c->space, c->step_dt * 0.5f);
  c->stepping = false;
}

// ---------- spawn driver ----------
static void spawn_loop_(Ctx* c) {
  if (!c || !c->alive || !c->space || c->stepping) return;
  if (c->inflight >= c->inflight_max) return;
  uint32_t now = now_ms();
  uint32_t due = 330 + (esp_random()%640);
  if (now - c->spawn_last_ms < due) return;
  c->spawn_last_ms = now;

  // near-full width; tiny safety margin
  int x = 2 + (esp_random() % std::max<int>(2, (int)c->W - 4));
  spawn_rocket_(c, x);
}

// ---------- init helpers ----------
static void prepare_canvas_(lv_obj_t* canvas, uint16_t W, uint16_t H) {
  lv_obj_remove_style_all(canvas);
  lv_obj_clear_flag(canvas, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scrollbar_mode(canvas, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_style_pad_all(canvas, 0, LV_PART_MAIN);
  lv_obj_set_size(canvas, W, H);
  lv_obj_set_pos(canvas, 0, 0);
}

static void create_space_(Ctx* c) {
  c->space = cpSpaceNew();
  cpSpaceSetIterations(c->space, 12);
  cpSpaceSetDamping(c->space, 0.993f);
  cpSpaceSetSleepTimeThreshold(c->space, 0.30f);
  cpSpaceSetGravity(c->space, cpv(0, GRAVITY_PX_S2));
}

// ---------- page-layer API ----------
static inline void on_load(lv_obj_t* layer) {
  if (!layer) return;
  Ctx* c = new Ctx();
  c->layer = layer;

  lv_obj_clear_flag(layer, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scrollbar_mode(layer, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_style_pad_all(layer, 0, LV_PART_MAIN);

  c->W = (uint16_t) (lv_obj_get_content_width(layer)  ? lv_obj_get_content_width(layer)  : lv_obj_get_width(layer));
  c->H = (uint16_t) (lv_obj_get_content_height(layer) ? lv_obj_get_content_height(layer) : lv_obj_get_height(layer));
  if (c->W == 0) c->W = 64;
  if (c->H == 0) c->H = 64;

  c->canvas = lv_canvas_create(layer);
  prepare_canvas_(c->canvas, c->W, c->H);

  c->buf = (uint8_t*) lv_mem_alloc(c->W * c->H * 2);
  lv_canvas_set_buffer(c->canvas, c->buf, c->W, c->H, LV_IMG_CF_TRUE_COLOR);
  lv_canvas_fill_bg(c->canvas, lv_color_black(), LV_OPA_COVER);

  lv_obj_set_user_data(layer, c);
  lv_obj_set_user_data(c->canvas, c);

  create_space_(c);

  c->tick = lv_timer_create([](lv_timer_t* t){
    Ctx* ctx = (Ctx*) t->user_data;
    if (!ctx || !ctx->alive) return;
    step_space_(ctx);
    render_(ctx);
  }, TICK_MS, c);

  c->spawner = lv_timer_create([](lv_timer_t* t){
    Ctx* ctx = (Ctx*) t->user_data;
    if (!ctx || !ctx->alive) return;
    spawn_loop_(ctx);
  }, SPAWN_MS, c);

  lv_obj_add_event_cb(c->canvas, [](lv_event_t* e){
    auto* cv = (lv_obj_t*) lv_event_get_target(e);
    auto* ctx = (Ctx*) lv_obj_get_user_data(cv);
    if (!ctx) return;
    ctx->alive = false;
    if (ctx->tick)    { lv_timer_del(ctx->tick);    ctx->tick=nullptr; }
    if (ctx->spawner) { lv_timer_del(ctx->spawner); ctx->spawner=nullptr; }
  }, LV_EVENT_DELETE, nullptr);
}

static inline void on_unload(lv_obj_t* layer) {
  if (!layer) return;
  Ctx* c = (Ctx*) lv_obj_get_user_data(layer);
  if (!c) return;

  c->alive = false;
  if (c->tick)    { lv_timer_del(c->tick);    c->tick=nullptr; }
  if (c->spawner) { lv_timer_del(c->spawner); c->spawner=nullptr; }

  clear_items_(c);

  if (c->canvas && lv_obj_is_valid(c->canvas)) {
    lv_obj_set_user_data(c->canvas, nullptr);
    lv_obj_del_async(c->canvas);
  }
  if (c->buf) { lv_mem_free(c->buf); c->buf = nullptr; }

  if (c->space) { cpSpaceFree(c->space); c->space = nullptr; }

  lv_obj_set_user_data(layer, nullptr);
  delete c;
}

// ---------- canvas-first API ----------
static inline void on_canvas_attach(lv_obj_t* canvas) {
  if (!canvas || !lv_obj_is_valid(canvas)) return;
  Ctx* c = new Ctx();
  c->canvas = canvas;
  c->layer  = lv_obj_get_parent(canvas);
  c->W = (uint16_t) lv_obj_get_width(canvas);
  c->H = (uint16_t) lv_obj_get_height(canvas);
  if (c->W == 0) c->W = 64;
  if (c->H == 0) c->H = 64;

  if (c->layer && lv_obj_is_valid(c->layer)) {
    lv_obj_clear_flag(c->layer, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(c->layer, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_pad_all(c->layer, 0, LV_PART_MAIN);
  }
  prepare_canvas_(canvas, c->W, c->H);

  c->buf = (uint8_t*) lv_mem_alloc(c->W * c->H * 2);
  lv_canvas_set_buffer(canvas, c->buf, c->W, c->H, LV_IMG_CF_TRUE_COLOR);
  lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);

  lv_obj_set_user_data(canvas, c);
  create_space_(c);

  c->tick = lv_timer_create([](lv_timer_t* t){
    Ctx* ctx = (Ctx*) t->user_data;
    if (!ctx || !ctx->alive) return;
    step_space_(ctx);
    render_(ctx);
  }, TICK_MS, c);

  c->spawner = lv_timer_create([](lv_timer_t* t){
    Ctx* ctx = (Ctx*) t->user_data;
    if (!ctx || !ctx->alive) return;
    spawn_loop_(ctx);
  }, SPAWN_MS, c);

  lv_obj_add_event_cb(canvas, [](lv_event_t* e){
    auto* cv = (lv_obj_t*) lv_event_get_target(e);
    auto* ctx = (Ctx*) lv_obj_get_user_data(cv);
    if (!ctx) return;
    ctx->alive = false;
    if (ctx->tick)    { lv_timer_del(ctx->tick);    ctx->tick=nullptr; }
    if (ctx->spawner) { lv_timer_del(ctx->spawner); ctx->spawner=nullptr; }
  }, LV_EVENT_DELETE, nullptr);
}

static inline void on_canvas_detach(lv_obj_t* canvas) {
  if (!canvas || !lv_obj_is_valid(canvas)) return;
  Ctx* c = (Ctx*) lv_obj_get_user_data(canvas);
  if (!c) return;

  c->alive = false;
  if (c->tick)    { lv_timer_del(c->tick);    c->tick=nullptr; }
  if (c->spawner) { lv_timer_del(c->spawner); c->spawner=nullptr; }

  clear_items_(c);

  if (c->buf) { lv_mem_free(c->buf); c->buf = nullptr; }
  lv_obj_set_user_data(canvas, nullptr);

  if (c->space) { cpSpaceFree(c->space); c->space = nullptr; }

  delete c;
}

} // namespace fireworks_canvas_detail

#ifdef __cplusplus
extern "C++" {
#endif
// Page-layer API
static inline void fireworks_physics_on_load(lv_obj_t* layer) {
  fireworks_canvas_detail::on_load(layer);
}
static inline void fireworks_physics_on_unload(lv_obj_t* layer) {
  fireworks_canvas_detail::on_unload(layer);
}
// Canvas-first API
static inline void fireworks_physics_attach_to_canvas(lv_obj_t* canvas) {
  fireworks_canvas_detail::on_canvas_attach(canvas);
}
static inline void fireworks_physics_detach_from_canvas(lv_obj_t* canvas) {
  fireworks_canvas_detail::on_canvas_detach(canvas);
}
#ifdef __cplusplus
}
#endif
