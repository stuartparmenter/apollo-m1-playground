#pragma once
#include <lvgl.h>
#include <math.h>

// ---------- LVGL v8 compatibility + safe async delete ----------
#ifndef LV_OBJ_FLAG_DELETE_PENDING
  #define LV_OBJ_FLAG_DELETE_PENDING LV_OBJ_FLAG_USER_1
#endif

#define FW_DEL_ASYNC_ONCE(_o) do {                                             \
  lv_obj_t* __o = (lv_obj_t*)(_o);                                             \
  if (__o && lv_obj_is_valid(__o) &&                                           \
      !lv_obj_has_flag(__o, LV_OBJ_FLAG_DELETE_PENDING)) {                     \
    lv_obj_add_flag(__o, LV_OBJ_FLAG_DELETE_PENDING);                          \
    lv_obj_del_async(__o);                                                     \
  }                                                                            \
} while(0)

namespace fireworks_detail {

struct FireCtx;
struct RocketParam { lv_timer_t* tail; FireCtx* ctx; };
struct PartParam  { lv_obj_t* obj; int x0,y0,x1,y1; bool gravity; };

struct FireCtx {
  lv_obj_t*  layer;
  lv_obj_t*  particles;        // container for ephemeral objects
  lv_timer_t* launcher_timer;
  bool       alive;
  uint8_t    inflight;
  uint8_t    inflight_max;

  // helpers
  void (*spawn_dot)(lv_obj_t*, int, int, lv_color_t, uint16_t, uint16_t);
  void (*spawn_particle)(lv_obj_t*, int, int, int, int, lv_color_t, uint16_t, uint16_t, uint16_t, bool);

  // colors
  lv_color_t (*pick_warm)();
  lv_color_t (*pick_accent)();
  lv_color_t (*tweak_color)(lv_color_t);

  // behavior
  void (*burst)(struct FireCtx*, int, int);
  void (*launch_rocket)(struct FireCtx*);
};

// ---------- on_load implementation ----------
static inline void fireworks_on_load(lv_obj_t* fw_layer) {
  if (!fw_layer) return;

  // Fresh layer styles & geometry (64x64)
  lv_obj_remove_style_all(fw_layer);
  lv_obj_set_size(fw_layer, 64, 64);
  lv_obj_set_pos(fw_layer, 0, 0);
  lv_obj_set_style_bg_opa(fw_layer, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(fw_layer, lv_color_black(), 0);
  lv_obj_clear_flag(fw_layer, LV_OBJ_FLAG_SCROLLABLE);

  // Allocate context
  FireCtx* ctx = (FireCtx*)lv_mem_alloc(sizeof(FireCtx));
  ctx->layer = fw_layer;
  ctx->launcher_timer = nullptr;
  ctx->alive = true;
  ctx->inflight = 0;
  ctx->inflight_max = 4;

  // Container for all ephemeral children
  ctx->particles = lv_obj_create(ctx->layer);
  lv_obj_remove_style_all(ctx->particles);
  lv_obj_clear_flag(ctx->particles, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(ctx->particles, 64, 64);
  lv_obj_set_pos(ctx->particles, 0, 0);

  // Stash ctx
  lv_obj_set_user_data(ctx->layer, ctx);
  lv_obj_set_user_data(ctx->particles, ctx);

  // If the container is deleted externally, stop spawns immediately
  lv_obj_add_event_cb(ctx->particles, [](lv_event_t* e){
    lv_obj_t* cont = (lv_obj_t*)lv_event_get_target(e);
    FireCtx* c = (FireCtx*)lv_obj_get_user_data(cont);
    if (!c) return;
    if (c->launcher_timer) { lv_timer_del(c->launcher_timer); c->launcher_timer = nullptr; }
    c->alive = false;
  }, LV_EVENT_DELETE, nullptr);

  // ---------- Colors ----------
  static const lv_color_t WARM[] = {
    lv_color_hex(0xFFFFFF),
    lv_color_hex(0xFFF4CC),
    lv_color_hex(0xFFD27F),
    lv_color_hex(0xFFB84D),
    lv_color_hex(0xFF9E2D)
  };
  static const lv_color_t ACCENT[] = {
    lv_color_hex(0x00E6E6),
    lv_color_hex(0x8A2BE2)
  };
  ctx->pick_warm = []() -> lv_color_t {
    return WARM[esp_random() % (sizeof(WARM)/sizeof(WARM[0]))];
  };
  ctx->pick_accent = []() -> lv_color_t {
    return ACCENT[esp_random() % (sizeof(ACCENT)/sizeof(ACCENT[0]))];
  };
  ctx->tweak_color = [](lv_color_t c) -> lv_color_t {
    uint8_t w = 16 + (esp_random() % 48);
    return lv_color_mix(lv_color_white(), c, w);
  };

  // ---------- Small fading dot ----------
  ctx->spawn_dot = [](lv_obj_t *parent, int x, int y, lv_color_t c, uint16_t size, uint16_t life_ms){
    lv_obj_t * d = lv_obj_create(parent);
    lv_obj_remove_style_all(d);
    lv_obj_clear_flag(d, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(d, size, size);
    lv_obj_set_pos(d, x, y);
    lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(d, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(d, c, 0);

    lv_anim_t af; lv_anim_init(&af);
    lv_anim_set_var(&af, d);
    lv_anim_set_values(&af, LV_OPA_COVER, LV_OPA_0);
    lv_anim_set_time(&af, life_ms);
    lv_anim_set_path_cb(&af, lv_anim_path_ease_out);
    lv_anim_set_exec_cb(&af, [](void * obj, int32_t v){
      lv_obj_set_style_opa((lv_obj_t*)obj, v, 0);
    });
    lv_anim_set_ready_cb(&af, [](lv_anim_t * a){
      FW_DEL_ASYNC_ONCE((lv_obj_t*)a->var);
    });
    lv_anim_start(&af);
  };

  // ---------- Particle with optional gravity ----------
  ctx->spawn_particle = [](lv_obj_t *parent, int x0, int y0, int x1, int y1,
                           lv_color_t c, uint16_t size, uint16_t move_ms,
                           uint16_t fade_delay_ms, bool gravity){
    lv_obj_t * p = lv_obj_create(parent);
    lv_obj_remove_style_all(p);
    lv_obj_clear_flag(p, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(p, size, size);
    lv_obj_set_pos(p, x0, y0);
    lv_obj_set_style_radius(p, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(p, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(p, c, 0);

    PartParam* prm = (PartParam*)lv_mem_alloc(sizeof(PartParam));
    prm->obj = p; prm->x0=x0; prm->y0=y0; prm->x1=x1; prm->y1=y1; prm->gravity=gravity;

    // Own and free PartParam on object delete; cancel movement anim
    lv_obj_set_user_data(p, prm);
    lv_obj_add_event_cb(p, [](lv_event_t* e){
      lv_obj_t* obj = (lv_obj_t*)lv_event_get_target(e);
      PartParam* prm2 = (PartParam*)lv_obj_get_user_data(obj);
      if (!prm2) return;
      lv_anim_del(prm2, NULL);      // cancel movement anims using prm2 as var
      lv_mem_free(prm2);
      lv_obj_set_user_data(obj, nullptr);
    }, LV_EVENT_DELETE, nullptr);

    // Movement anim uses PartParam* as var (not the object)
    lv_anim_t a; lv_anim_init(&a);
    lv_anim_set_var(&a, prm);
    lv_anim_set_values(&a, 0, 1000);
    lv_anim_set_time(&a, move_ms);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_exec_cb(&a, [](void * vprm, int32_t t){
      PartParam* q = (PartParam*)vprm;
      if (!q || !q->obj) return;
      float u = t / 1000.0f;
      float s = u*u*(3 - 2*u);
      float x = q->x0 + (q->x1 - q->x0) * s;
      float y = q->y0 + (q->y1 - q->y0) * s;
      if (q->gravity) y += 10.0f * (u*u);
      lv_obj_set_pos(q->obj, (int)x, (int)y);
    });
    lv_anim_start(&a);

    // Fade tied to the object; delete asynchronously
    uint16_t fadetime = move_ms/2; if (fadetime<200) fadetime=200; if (fadetime>500) fadetime=500;
    lv_anim_t af; lv_anim_init(&af);
    lv_anim_set_var(&af, p);
    lv_anim_set_values(&af, LV_OPA_COVER, LV_OPA_0);
    lv_anim_set_time(&af, fadetime);
    lv_anim_set_delay(&af, fade_delay_ms);
    lv_anim_set_exec_cb(&af, [](void * obj, int32_t v){
      lv_obj_set_style_opa((lv_obj_t*)obj, v, 0);
    });
    lv_anim_set_ready_cb(&af, [](lv_anim_t * a3){
      FW_DEL_ASYNC_ONCE((lv_obj_t*)a3->var);
    });
    lv_anim_start(&af);
  };

  // ---------- Burst patterns ----------
  enum Burst { PEONY=0, RING=1, WILLOW=2, CHRYS=3 };
  ctx->burst = [](FireCtx* c, int cx, int cy){
    bool accent = (esp_random() % 6 == 0);
    lv_color_t base = accent ? c->pick_accent() : c->pick_warm();

    int type_roll = esp_random() % 100;
    Burst type = PEONY;
    if (type_roll < 15)      type = RING;
    else if (type_roll < 35) type = WILLOW;
    else if (type_roll < 55) type = CHRYS;

    int n  = 14 + (esp_random() % 10);
    int R  = 16 + (esp_random() % 13);
    int T  = 550 + (esp_random() % 250);
    int S  = 1 + (esp_random() % 2);
    bool sparkle = (esp_random() % 3 == 0);

    auto on_canvas_or_near = [](int x, int y){
      return (x >= -2 && x <= 65 && y >= -2 && y <= 65);
    };

    for (int i = 0; i < n; i++) {
      float a = (2.0f * M_PI * i) / n;
      float r = (float)R;
      if (type == CHRYS) r += 2.0f * sinf(a * 8.0f + (esp_random() % 100) * 0.01f);

      int x_to = cx + (int)(cosf(a) * r);
      int y_to = cy + (int)(sinf(a) * r);

      if (type == RING) {
        x_to = cx + (int)(cosf(a) * (R + 1));
        y_to = cy + (int)(sinf(a) * (R + 1));
      } else if (type == WILLOW) {
        y_to += 6;
      }

      // Cull only if both ends offscreen
      if (!on_canvas_or_near(cx, cy) && !on_canvas_or_near(x_to, y_to)) continue;
      lv_color_t col = c->tweak_color(base);
      c->spawn_particle(c->particles, cx, cy, x_to, y_to, col, S, T, T*3/5, (type==WILLOW));
    }

    if (sparkle) {
      int k = 6 + (esp_random() % 6);
      for (int j=0; j<k; j++){
        int rx = cx + (int)((esp_random()% (R*2)) - R);
        int ry = cy + (int)((esp_random()% (R*2)) - R);
        if (!on_canvas_or_near(rx, ry)) continue;
        c->spawn_dot(c->particles, rx, ry,
                     lv_color_mix(lv_color_white(), base, 64),
                     1, 220 + (esp_random()%180));
      }
    }
  };

  // ---------- Rocket + tail (safe timer ownership) ----------
  ctx->launch_rocket = [](FireCtx* c){
    if (!c->alive) return;
    if (c->inflight >= c->inflight_max) return;

    int x = 6 + (esp_random() % 52);
    int apex_y = 10 + (esp_random() % 28);
    int start_y = 63;

    lv_obj_t * r = lv_obj_create(c->particles);
    lv_obj_remove_style_all(r);
    lv_obj_clear_flag(r, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(r, 1, 1);
    lv_obj_set_pos(r, x, start_y);

    RocketParam* rp = (RocketParam*)lv_mem_alloc(sizeof(RocketParam));
    rp->tail = nullptr; rp->ctx = c;
    lv_obj_set_user_data(r, rp);

    c->inflight++;

    // Delete handler: detach timer only (no lv_timer_del here)
    lv_obj_add_event_cb(r, [](lv_event_t* e){
      lv_obj_t* rocket = (lv_obj_t*)lv_event_get_target(e);
      RocketParam* rp2 = (RocketParam*)lv_obj_get_user_data(rocket);
      FireCtx* c2 = rp2 ? rp2->ctx : nullptr;

      if (rp2 && rp2->tail) {
        rp2->tail->user_data = nullptr;  // tell callback to self-delete
        rp2->tail = nullptr;
      }

      if (rp2) { lv_mem_free(rp2); lv_obj_set_user_data(rocket, nullptr); }
      if (c2 && c2->inflight > 0) c2->inflight--;
    }, LV_EVENT_DELETE, nullptr);

    // Rise animation
    uint16_t rise_ms = 450 + (esp_random()%250);
    lv_anim_t a; lv_anim_init(&a);
    lv_anim_set_var(&a, r);
    lv_anim_set_values(&a, start_y, apex_y);
    lv_anim_set_time(&a, rise_ms);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_exec_cb(&a, [](void * obj, int32_t v){
      lv_obj_set_y((lv_obj_t*)obj, v);
    });
    lv_anim_set_ready_cb(&a, [](lv_anim_t * a1){
      lv_obj_t * rocket = (lv_obj_t*)a1->var;
      if (!rocket) return;

      lv_obj_t * particles = lv_obj_get_parent(rocket);
      FireCtx* c = particles ? (FireCtx*)lv_obj_get_user_data(particles) : nullptr;
      if (!particles || !c) { FW_DEL_ASYNC_ONCE(rocket); return; }

      lv_coord_t rx = lv_obj_get_x(rocket);
      lv_coord_t ry = lv_obj_get_y(rocket);

      // Flash at apex
      lv_obj_t * flash = lv_obj_create(particles);
      lv_obj_remove_style_all(flash);
      lv_obj_set_size(flash, 4, 4);
      lv_obj_set_pos(flash, rx-1, ry-1);
      lv_obj_set_style_radius(flash, LV_RADIUS_CIRCLE, 0);
      lv_obj_set_style_bg_color(flash, lv_color_hex(0xFFF4CC), 0);
      lv_obj_set_style_bg_opa(flash, LV_OPA_COVER, 0);
      lv_anim_t ff; lv_anim_init(&ff);
      lv_anim_set_var(&ff, flash);
      lv_anim_set_values(&ff, 255, 0);
      lv_anim_set_time(&ff, 140);
      lv_anim_set_exec_cb(&ff, [](void * obj, int32_t v){
        lv_obj_set_style_opa((lv_obj_t*)obj, v, 0);
      });
      lv_anim_set_ready_cb(&ff, [](lv_anim_t * a2){
        FW_DEL_ASYNC_ONCE((lv_obj_t*)a2->var);
      });
      lv_anim_start(&ff);

      if (c->alive && c->burst) c->burst(c, rx, ry);

      FW_DEL_ASYNC_ONCE(rocket);
    });
    lv_anim_start(&a);

    // Tail timer: user_data = rocket; self-deletes if rocket missing/invalid
    RocketParam* rp_for_timer = (RocketParam*)lv_obj_get_user_data(r);
    rp_for_timer->tail = lv_timer_create([](lv_timer_t * t){
      lv_obj_t * rocket = t ? (lv_obj_t*)t->user_data : nullptr;
      if (!rocket || !lv_obj_is_valid(rocket)) { lv_timer_del(t); return; }

      lv_obj_t * particles = lv_obj_get_parent(rocket);
      if (!particles) { lv_timer_del(t); return; }

      int rx = lv_obj_get_x(rocket);
      int ry = lv_obj_get_y(rocket);

      lv_color_t col = (esp_random()%3==0) ? lv_color_hex(0xFFB84D) : lv_color_hex(0xFFFFFF);
      int life = 120 + (esp_random()%100);
      int sx = rx + (int)((esp_random()%3)-1);
      int sy = ry + 1 + (esp_random()%2);

      if (sx < 0 || sx > 63 || sy < 0 || sy > 63) return;

      lv_obj_t * d = lv_obj_create(particles);
      lv_obj_remove_style_all(d);
      lv_obj_set_size(d, 1, 1);
      lv_obj_set_pos(d, sx, sy);
      lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, 0);
      lv_obj_set_style_bg_color(d, col, 0);
      lv_obj_set_style_bg_opa(d, LV_OPA_COVER, 0);

      lv_anim_t af; lv_anim_init(&af);
      lv_anim_set_var(&af, d);
      lv_anim_set_values(&af, 255, 0);
      lv_anim_set_time(&af, life);
      lv_anim_set_exec_cb(&af, [](void * obj, int32_t v){
        lv_obj_set_style_opa((lv_obj_t*)obj, v, 0);
      });
      lv_anim_set_ready_cb(&af, [](lv_anim_t * a2){
        FW_DEL_ASYNC_ONCE((lv_obj_t*)a2->var);
      });
      lv_anim_start(&af);

    }, 40, r);
  };

  // ---------- Periodic launcher (throttled) ----------
  ctx->launcher_timer = lv_timer_create([](lv_timer_t * t){
    FireCtx* c = (FireCtx*)t->user_data;
    if (!c || !c->particles || !c->alive) return;

    static uint32_t next_gap = 0;
    static uint32_t last_ms = 0;
    uint32_t now = lv_tick_get();
    if (next_gap == 0) next_gap = 350 + (esp_random()%450);

    if (now - last_ms >= next_gap) {
      if (c->inflight < c->inflight_max) {
        int burst_count = (esp_random()%5==0) ? 2 : 1;
        for (int i=0;i<burst_count;i++){
          if (c->launch_rocket) c->launch_rocket(c);
        }
      }
      last_ms = now;
      next_gap = 300 + (esp_random()%700);
    }
  }, 40, ctx);
}

// ---------- on_unload implementation ----------
static inline void fireworks_on_unload(lv_obj_t* fw_layer) {
  if (!fw_layer) return;

  // Mirror FireCtx layout to fetch the pointer cleanly
  FireCtx* ctx = (FireCtx*)lv_obj_get_user_data(fw_layer);
  if (!ctx) return;

  // Stop new activity
  ctx->alive = false;

  // Stop the launcher timer
  if (ctx->launcher_timer) {
    lv_timer_del(ctx->launcher_timer);
    ctx->launcher_timer = nullptr;
  }

  // Avoid global lv_anim_del_all(); let container deletion cascade clean up.
  if (ctx->particles && lv_obj_is_valid(ctx->particles)) {
    FW_DEL_ASYNC_ONCE(ctx->particles);
  }

  lv_mem_free(ctx);
  lv_obj_set_user_data(fw_layer, nullptr);
}

} // namespace fireworks_detail


#ifdef __cplusplus
extern "C++" {
#endif

// Public wrappers to keep YAML clean
static inline void fireworks_on_load(lv_obj_t* fw_layer) {
  fireworks_detail::fireworks_on_load(fw_layer);
}
static inline void fireworks_on_unload(lv_obj_t* fw_layer) {
  fireworks_detail::fireworks_on_unload(fw_layer);
}

#ifdef __cplusplus
}
#endif

