// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// Fireworks + Chipmunk2D on LVGL canvas (ESP-IDF, LVGL v8+)

#pragma once

extern "C" {
#include <lvgl.h>
}

#include <stdint.h>
#include <math.h>
#include <vector>
#include <algorithm>

#include <chipmunk/chipmunk.h>

namespace fireworks_canvas_detail {

//============================== Tunables ==============================//

static constexpr uint8_t  TRAIL_FADE_OPA   = 32;     // lower = longer trails
static constexpr uint32_t TICK_MS          = 33;     // ~30 FPS
static constexpr int      CULL_MARGIN      = 2;      // overscan margin
static constexpr uint32_t HARD_TTL_MS      = 3200;   // absolute max particle life
static constexpr uint32_t SOFT_TTL_PAD_MS  = 800;    // extra grace beyond requested life
static constexpr uint32_t MIN_SPAWN_GAP_MS = 300;    // min ms between core spawns
static constexpr uint8_t  MAX_INFLIGHT     = 3;      // cap concurrent rockets

//============================== Small helpers ==============================//

static inline uint32_t now_ms() { return lv_tick_get(); }

static inline bool obj_alive_(lv_obj_t* o) {
  if (!o) return false;
  if (!lv_obj_is_valid(o)) return false;
#ifdef LV_OBJ_FLAG_DELETE_PENDING
  if (lv_obj_has_flag(o, LV_OBJ_FLAG_DELETE_PENDING)) return false;
#endif
  return true;
}

static inline float frand(float a, float b) {
  return a + (b - a) * (float)(esp_random() & 0xFFFF) / 65535.0f;
}

static inline lv_color_t pick_warm() {
  static const lv_color_t WARM[] = {
    lv_color_hex(0xFFFFFF), lv_color_hex(0xFFF4CC), lv_color_hex(0xFFD27F),
    lv_color_hex(0xFFB84D), lv_color_hex(0xFF9E2D)
  };
  return WARM[esp_random() % (sizeof(WARM)/sizeof(WARM[0]))];
}
static inline lv_color_t pick_accent() {
  static const lv_color_t ACCENT[] = {
    lv_color_hex(0x00E6E6), lv_color_hex(0x8A2BE2)
  };
  return ACCENT[esp_random() % (sizeof(ACCENT)/sizeof(ACCENT[0]))];
}
static inline lv_color_t tweak(lv_color_t c) {
  uint8_t w = 16 + (esp_random() % 48);
  return lv_color_mix(lv_color_white(), c, w);
}

//============================== Physics objects ==============================//

struct BodyRef {
  cpBody*    body{nullptr};
  bool       is_rocket{false};
  bool       alive{true};

  lv_color_t color{lv_color_white()};
  uint8_t    px{1};
  uint32_t   born_ms{0};
  uint32_t   life_ms{1000};

  // rocket-only
  uint32_t   fuse_ms{0};

  // crossette-only
  bool       can_split{false};
  bool       split_done{false};
  uint32_t   split_ms{0};
};

// Free body safely (detach if in space)
static inline void free_body_(cpBody* b) {
  if (!b) return;
  if (cpSpace* s = cpBodyGetSpace(b)) cpSpaceRemoveBody(s, b);
  cpBodyFree(b);
}

//============================== Context ==============================//

struct Ctx {
  // Canvas + buffer (owned)
  lv_obj_t*  canvas{nullptr};
  uint8_t*   buf{nullptr};
  uint16_t   W{0}, H{0};

  // Single timer + debounced resize + liveness
  lv_timer_t* tick{nullptr};
  lv_timer_t* resize_debounce{nullptr};
  bool        running{false};
  bool        alive{true};

  // Chipmunk space
  cpSpace* space{nullptr};

  // Particles
  std::vector<BodyRef*> items;

  // Spawn control (single-loop)
  uint8_t  inflight{0};
  uint32_t last_spawn_ms{0};

  // Visuals
  uint8_t  fade_opa{TRAIL_FADE_OPA};

  // Size-aware parameters (recomputed on resize)
  float    grav_px_s2{180.0f};   // px/s^2
  float    launch_vy_min{-220.0f};
  float    launch_vy_max{-180.0f};
  float    launch_vx_abs{15.0f};
  uint32_t burst_t_min_ms{600};
  uint32_t burst_t_max_ms{950};
  float    frag_speed_min{55.0f};
  float    frag_speed_max{85.0f};
};

//============================== Drawing ==============================//

static inline void draw_px_rect_(lv_obj_t* canvas, int x, int y, uint8_t px, lv_color_t color) {
  if (!obj_alive_(canvas)) return;
  const int W = lv_obj_get_width(canvas), H = lv_obj_get_height(canvas);
  if (x >= W || y >= H || x + (int)px <= 0 || y + (int)px <= 0) return;

  const int sx = (x < 0) ? 0 : x;
  const int sy = (y < 0) ? 0 : y;
  const int ex = std::min(W, x + (int)px);
  const int ey = std::min(H, y + (int)px);

  lv_draw_rect_dsc_t d; lv_draw_rect_dsc_init(&d);
  d.bg_color   = color;
  d.bg_opa     = LV_OPA_COVER;
  d.border_opa = LV_OPA_TRANSP;
  lv_canvas_draw_rect(canvas, sx, sy, ex - sx, ey - sy, &d);
}

static inline void cull_and_free_(std::vector<BodyRef*>& items, int W, int H, Ctx* owner = nullptr) {
  const uint32_t t = now_ms();
  items.erase(std::remove_if(items.begin(), items.end(), [&](BodyRef* br){
    if (!br) return true;
    const bool too_old = (t - br->born_ms > (br->life_ms + SOFT_TTL_PAD_MS)) ||
                         (t - br->born_ms > HARD_TTL_MS);
    const cpVect p = br->body ? cpBodyGetPosition(br->body) : cpv(-9999, -9999);
    const bool oob = (p.x < -CULL_MARGIN) || (p.y < -CULL_MARGIN) ||
                     (p.x > W + CULL_MARGIN) || (p.y > H + CULL_MARGIN);
    if (too_old || oob || !br->alive) {
      if (owner && br->is_rocket && owner->inflight > 0) owner->inflight--;
      free_body_(br->body);
      delete br;
      return true;
    }
    return false;
  }), items.end());
}

static inline void render_(Ctx* c) {
  if (!c || !obj_alive_(c->canvas) || !c->buf || c->W == 0 || c->H == 0) return;

  // Trails / motion blur
  lv_canvas_fill_bg(c->canvas, lv_color_black(), c->fade_opa);

  for (auto* br : c->items) {
    if (!br || !br->body) continue;
    const cpVect p = cpBodyGetPosition(br->body);
    draw_px_rect_(c->canvas, (int)p.x, (int)p.y, br->px, br->color);
  }

  cull_and_free_(c->items, c->W, c->H, c);
}

//============================== Space / Items ==============================//

static inline cpBody* make_body_(cpSpace* space, float x, float y, float vx, float vy, float mass=1.0f) {
  cpBody* body = cpBodyNew((cpFloat)mass, (cpFloat)INFINITY);  // no rotation
  cpBodySetPosition(body, cpv((cpFloat)x,(cpFloat)y));
  cpBodySetVelocity(body, cpv((cpFloat)vx,(cpFloat)vy));
  cpSpaceAddBody(space, body);
  return body;
}

static inline void create_space_(Ctx* c) {
  if (!c || c->space) return;
  c->space = cpSpaceNew();
  cpSpaceSetIterations(c->space, 12);
  cpSpaceSetDamping(c->space, (cpFloat)0.993);
  cpSpaceSetSleepTimeThreshold(c->space, (cpFloat)0.30);
  cpSpaceSetGravity(c->space, cpv((cpFloat)0, (cpFloat)c->grav_px_s2));
}

static inline void clear_items_(Ctx* c) {
  if (!c) return;
  for (auto* br : c->items) { if (br) { free_body_(br->body); delete br; } }
  c->items.clear();
  c->inflight = 0;
}

//============================== Size-aware parameterization ==============================//

static inline void recompute_params_(Ctx* c) {
  if (!c || c->H == 0) return;

  // Gravity proportional to height → consistent look across sizes.
  // Baseline ~180 px/s^2 at 64px tall ⇒ ≈ 2.8125 * H
  const float g = 2.8125f * (float)c->H;  // px/s^2
  c->grav_px_s2 = g;

  // Target apex ~62% up from launch y ⇒ v0 = sqrt(2 g Δh)
  const float delta_h = 0.62f * (float)c->H;
  const float v0      = sqrtf(2.0f * g * delta_h);     // upward magnitude

  // Launch scatter around v0
  c->launch_vy_min = -1.10f * v0;
  c->launch_vy_max = -0.90f * v0;

  // Tiny horizontal wobble scales with height
  c->launch_vx_abs = 0.06f * (float)c->H;

  // Burst timing around apex t = v0/g (ms)
  const float t_apex_s = v0 / g;
  const uint32_t t_ms  = (uint32_t)(t_apex_s * 1000.0f);
  c->burst_t_min_ms    = (uint32_t)(0.90f * t_ms);
  c->burst_t_max_ms    = (uint32_t)(1.10f * t_ms);

  // Fragment speeds scale with launch speed → consistent diameter
  c->frag_speed_min    = 0.55f * v0;
  c->frag_speed_max    = 0.95f * v0;
}

//============================== Spawning & Bursting (single-loop) ==============================//

static inline BodyRef* spawn_particle_(Ctx* c, float x, float y, float vx, float vy,
                                       lv_color_t base, uint8_t px=1, uint32_t life_ms=1000,
                                       bool can_split=false, uint32_t split_ms=0) {
  if (!c || !c->space) return nullptr;
  BodyRef* br = new BodyRef();
  br->is_rocket = false;
  br->born_ms   = now_ms();
  br->life_ms   = life_ms;
  br->px        = px;
  br->color     = tweak(base);
  br->can_split = can_split;
  br->split_ms  = split_ms;

  br->body = make_body_(c->space, x, y, vx, vy, 1.0f);
  c->items.push_back(br);
  return br;
}

static inline BodyRef* spawn_core_(Ctx* c, float cx, float cy, uint32_t life_ms=1500, uint8_t px=2) {
  if (!c || !c->space) return nullptr;
  const float vx = frand(-c->launch_vx_abs, c->launch_vx_abs);
  const float vy = frand(c->launch_vy_min, c->launch_vy_max);

  BodyRef* core = new BodyRef();
  core->is_rocket = true;
  core->born_ms   = now_ms();
  core->fuse_ms   = (uint32_t)frand(0.85f, 1.15f) * (c->burst_t_min_ms + c->burst_t_max_ms)/2;
  core->life_ms   = life_ms;
  core->px        = px;
  core->color     = lv_color_white();

  core->body = make_body_(c->space, cx, cy, vx, vy, 1.0f);
  c->items.push_back(core);
  c->inflight++;
  return core;
}

// ----- Burst patterns -----
enum class BurstKind : uint8_t { PEONY, CHRYS, RING, STAR5, STAR6, PALM, WILLOW, CROSSETTE };

static inline BurstKind pick_burst_kind() {
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

static inline void burst_peony_(Ctx* c, int cx, int cy, bool chrys=false) {
  bool accent = (frand(0,1) < 1.0f/6.0f);
  lv_color_t base = accent ? pick_accent() : pick_warm();
  const int   n   = 20 + (int)frand(0,18);
  const float s0  = frand(c->frag_speed_min*0.8f, c->frag_speed_max*0.85f);
  const int   life= chrys ? (1100 + (int)frand(0,600)) : (800 + (int)frand(0,400));
  for (int i=0;i<n;i++) {
    float a = (2.0f * (float)M_PI * i) / n;
    float jitter = frand(-0.6f, 0.6f) * 0.02f;
    float sp = s0 * frand(0.90f, 1.10f);
    float vx = cosf(a + jitter) * sp;
    float vy = sinf(a + jitter) * sp;
    uint8_t px = (frand(0,1) < 0.25f) ? 2 : 1;
    spawn_particle_(c, (float)cx, (float)cy, vx, vy, base, px, (uint32_t)life);
  }
}
static inline void burst_ring_(Ctx* c, int cx, int cy) {
  bool accent = (frand(0,1) < 0.33f);
  lv_color_t base = accent ? pick_accent() : pick_warm();
  const int   n   = 26 + (int)frand(0,16);
  const float s   = frand(c->frag_speed_min*0.9f, c->frag_speed_max*0.9f);
  const int   life= 850 + (int)frand(0,450);
  for (int i=0;i<n;i++) {
    float a = (2.0f * (float)M_PI * i) / n;
    spawn_particle_(c, (float)cx, (float)cy, cosf(a)*s, sinf(a)*s, base, 1, (uint32_t)life);
  }
}
static inline void burst_starN_(Ctx* c, int cx, int cy, int N) {
  lv_color_t base = pick_warm();
  const float s   = frand(c->frag_speed_min*0.95f, c->frag_speed_max);
  const int   life= 900 + (int)frand(0,400);
  for (int i=0;i<N;i++) {
    float a = (2.0f * (float)M_PI * i) / N;
    int rays = 3 + (int)frand(0,1);
    for (int k=0;k<rays;k++) {
      float t = 0.55f + 0.18f * k; // 55%, 73%, 91%
      spawn_particle_(c, (float)cx, (float)cy, cosf(a)*s*t, sinf(a)*s*t, base, 1, (uint32_t)life);
    }
  }
  burst_ring_(c, cx, cy);
}
static inline void burst_palm_(Ctx* c, int cx, int cy) {
  lv_color_t base = pick_warm();
  const int fronds = 8 + (int)frand(0,6);
  const int life   = 1000 + (int)frand(0,500);
  for (int i=0;i<fronds;i++) {
    float spread = (35.0f * (float)M_PI/180.0f);
    float a = -((float)M_PI/2) + frand(-spread, spread);
    float s = frand(c->frag_speed_min*1.0f, c->frag_speed_max*1.05f);
    spawn_particle_(c, (float)cx, (float)cy, cosf(a)*s, sinf(a)*s, base, 2, (uint32_t)life);
  }
}
static inline void burst_willow_(Ctx* c, int cx, int cy) {
  lv_color_t base = pick_warm();
  const int   n   = 18 + (int)frand(0,10);
  const float s   = frand(c->frag_speed_min*0.65f, c->frag_speed_min*0.85f);
  const int   life= 1600 + (int)frand(0,800);
  for (int i=0;i<n;i++) {
    float a = (2.0f * (float)M_PI * i) / n;
    float vx = cosf(a) * s, vy = sinf(a) * s + 2.0f; // gentle droop
    spawn_particle_(c, (float)cx, (float)cy, vx, vy, base, 1, (uint32_t)life);
  }
}
static inline void burst_crossette_(Ctx* c, int cx, int cy) {
  lv_color_t base = pick_warm();
  const int   n   = 10 + (int)frand(0,6);
  const float s   = frand(c->frag_speed_min*0.9f, c->frag_speed_min*1.2f);
  const int   life= 1100 + (int)frand(0,300);
  for (int i=0;i<n;i++) {
    float a = (2.0f * (float)M_PI * i) / n;
    const uint32_t split_ms = 240 + (uint32_t)frand(0,220);
    spawn_particle_(c, (float)cx, (float)cy, cosf(a)*s, sinf(a)*s, base, 1, (uint32_t)life,
                    /*can_split=*/true, split_ms);
  }
}
static inline void burst_dispatch_(Ctx* c, int cx, int cy, BurstKind k) {
  switch (k) {
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

// Crossette secondary split (handled pre-step within the single loop)
static inline void handle_crossette_splits_(Ctx* c, uint32_t tnow) {
  std::vector<BodyRef*> to_split;
  to_split.reserve(8);
  for (auto* it : c->items) {
    if (!it || it->is_rocket || !it->body || it->split_done || !it->can_split) continue;
    if (tnow - it->born_ms >= it->split_ms) to_split.push_back(it);
  }
  for (auto* it : to_split) {
    const cpVect p = cpBodyGetPosition(it->body);
    const float sv = frand(c->frag_speed_min*0.55f, c->frag_speed_min*0.75f);
    spawn_particle_(c, (float)p.x, (float)p.y,  sv,  0, it->color, 1, 600);
    spawn_particle_(c, (float)p.x, (float)p.y, -sv,  0, it->color, 1, 600);
    spawn_particle_(c, (float)p.x, (float)p.y,  0,  sv, it->color, 1, 600);
    spawn_particle_(c, (float)p.x, (float)p.y,  0, -sv, it->color, 1, 600);
    it->split_done = true;
    it->alive = false; // retire original shard
  }
}

// Rockets that reach fuse burst into one of the patterns
static inline void handle_rocket_bursts_(Ctx* c, uint32_t tnow) {
  std::vector<BodyRef*> to_burst;
  to_burst.reserve(4);
  for (auto* it : c->items) {
    if (!it || !it->is_rocket || !it->body) continue;
    if (tnow - it->born_ms >= it->fuse_ms) to_burst.push_back(it);
  }
  for (auto* it : to_burst) {
    const cpVect p = cpBodyGetPosition(it->body);
    burst_dispatch_(c, (int)p.x, (int)p.y, pick_burst_kind());
    it->alive = false;
    if (c->inflight) c->inflight--;
  }
}

// Simple cadence to inject new cores (no second timer needed)
static inline void maybe_spawn_(Ctx* c, uint32_t tnow) {
  if (c->inflight >= MAX_INFLIGHT) return;
  // Scale gap with screen/time-to-apex so big canvases don't get too "busy"
  const uint32_t scaled_gap = std::max<uint32_t>(
      MIN_SPAWN_GAP_MS,
      (c->burst_t_min_ms + c->burst_t_max_ms) / 3  // ~0.33 * time-to-apex
  );
  if (tnow - c->last_spawn_ms < scaled_gap) return;

  const float pad = 8.0f;
  const float cx = frand(pad, std::max(pad, (float)c->W - pad));
  const float cy = (float)c->H - 2.0f; // near ground
  spawn_core_(c, cx, cy, 1300 + (uint32_t)frand(0, 500), 2);
  c->last_spawn_ms = tnow;
}

//============================== Lifecycle helpers ==============================//

static inline void ctx_stop_timer_(Ctx* c){
  if (!c) return;
  if (c->tick) { lv_timer_del(c->tick); c->tick = nullptr; }
  c->running = false;
}

static inline void ctx_free_canvas_buf_(Ctx* c){
  if (!c) return;
  if (c->buf) { lv_mem_free(c->buf); c->buf = nullptr; }
}

// Single-loop timer: mutate → step → render
static inline void ctx_start_timer_(Ctx* c){
  if (!c || c->running) return;

  c->tick = lv_timer_create([](lv_timer_t* t){
    auto* c = (Ctx*) t->user_data;
    if (!c || !c->alive || !obj_alive_(c->canvas) || !c->buf || c->W == 0 || c->H == 0 || !c->space) return;

    const uint32_t tnow = now_ms();

    // 1) pre-step mutations
    handle_crossette_splits_(c, tnow);
    handle_rocket_bursts_(c, tnow);
    maybe_spawn_(c, tnow);

    // 2) step physics
    cpSpaceSetGravity(c->space, cpv((cpFloat)0, (cpFloat)c->grav_px_s2));
    cpSpaceStep(c->space, (cpFloat)TICK_MS / (cpFloat)1000.0);

    // 3) render & cull
    render_(c);
  }, TICK_MS, c);

  c->running = true;
}

// Resize/start: allocate new buffer; set it; free old; recompute physics; (re)start timer
static inline void ctx_resize_start_(Ctx* c, uint16_t w, uint16_t h){
  if (!c || !obj_alive_(c->canvas) || w==0 || h==0) return;

  if (c->buf && c->W == w && c->H == h) { // size unchanged
    if (!c->running) ctx_start_timer_(c);
    return;
  }

  ctx_stop_timer_(c);
  clear_items_(c);

  uint8_t* newbuf = (uint8_t*) lv_mem_alloc((size_t)w * h * 2 /*RGB565*/);
  if (!newbuf) return; // OOM: keep previous state if any

  // Respect external layout; we only swap buffers
  lv_canvas_set_buffer(c->canvas, newbuf, w, h, LV_IMG_CF_TRUE_COLOR);
  lv_canvas_fill_bg(c->canvas, lv_color_black(), LV_OPA_COVER);

  uint8_t* old = c->buf; c->buf = newbuf;
  c->W = w; c->H = h;
  if (old) lv_mem_free(old);

  if (!c->space) create_space_(c);

  recompute_params_(c);
  ctx_start_timer_(c);
}

// Debounced size-commit: read current size and (re)start once per burst
static void resize_commit_cb_(lv_timer_t* t){
  auto* c = (Ctx*) t->user_data;
  if (!c || !c->alive || !obj_alive_(c->canvas)) { if (t) lv_timer_del(t); return; }
  c->resize_debounce = nullptr;

  lv_obj_update_layout(c->canvas);
  const uint16_t w = (uint16_t) lv_obj_get_width(c->canvas);
  const uint16_t h = (uint16_t) lv_obj_get_height(c->canvas);
  ctx_resize_start_(c, w, h);
}

//============================== Attach / Detach ==============================//

static inline void on_canvas_attach(lv_obj_t* canvas) {
  if (!canvas || !lv_obj_is_valid(canvas)) return;

  Ctx* c = new Ctx();
  c->canvas = canvas;
  lv_obj_set_user_data(canvas, c);

  // Debounced SIZE_CHANGED to coalesce layout bursts
  lv_obj_add_event_cb(canvas, [](lv_event_t* e){
    if (lv_event_get_code(e) != LV_EVENT_SIZE_CHANGED) return;
    auto* c = (Ctx*) lv_event_get_user_data(e);
    if (!c || !c->alive) return;
    if (c->resize_debounce) lv_timer_reset(c->resize_debounce);
    else c->resize_debounce = lv_timer_create(resize_commit_cb_, 20, c);
  }, LV_EVENT_SIZE_CHANGED, c);

  // Teardown on DELETE (mirrors explicit detach)
  lv_obj_add_event_cb(canvas, [](lv_event_t* e){
    if (lv_event_get_code(e) != LV_EVENT_DELETE) return;
    auto* cv = (lv_obj_t*) lv_event_get_target(e);
    auto* c  = (Ctx*) lv_obj_get_user_data(cv);
    if (!c) return;
    c->alive = false;
    if (c->resize_debounce) { lv_timer_del(c->resize_debounce); c->resize_debounce = nullptr; }
    if (c->tick) { lv_timer_del(c->tick); c->tick = nullptr; }
    clear_items_(c);
    if (c->buf) { lv_mem_free(c->buf); c->buf = nullptr; }
    if (c->space) { cpSpaceFree(c->space); c->space = nullptr; }
    lv_obj_set_user_data(cv, nullptr);
    c->canvas = nullptr;
    delete c;
  }, LV_EVENT_DELETE, nullptr);

  // Bootstrap: create space and buffer through the same debounced path
  create_space_(c);
  c->resize_debounce = lv_timer_create(resize_commit_cb_, 1, c);
}

static inline void on_canvas_detach(lv_obj_t* canvas) {
  if (!canvas || !lv_obj_is_valid(canvas)) return;
  Ctx* c = (Ctx*) lv_obj_get_user_data(canvas);
  if (!c) return;

  c->alive = false;
  if (c->resize_debounce) { lv_timer_del(c->resize_debounce); c->resize_debounce = nullptr; }
  if (c->tick) { lv_timer_del(c->tick); c->tick = nullptr; }
  clear_items_(c);
  if (c->buf) { lv_mem_free(c->buf); c->buf = nullptr; }
  if (c->space) { cpSpaceFree(c->space); c->space = nullptr; }
  lv_obj_set_user_data(canvas, nullptr);
  c->canvas = nullptr;
  delete c;
}

} // namespace fireworks_canvas_detail

#ifdef __cplusplus
extern "C++" {
#endif

// Public Canvas-first API
static inline void fireworks_physics_attach_to_canvas(lv_obj_t* canvas) {
  fireworks_canvas_detail::on_canvas_attach(canvas);
}
static inline void fireworks_physics_detach_from_canvas(lv_obj_t* canvas) {
  fireworks_canvas_detail::on_canvas_detach(canvas);
}

#ifdef __cplusplus
}
#endif
