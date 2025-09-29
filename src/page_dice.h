// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// src/page_dice.h — D6 toss using pure animation system
// Renders a 3D-looking die in LVGL, uses deterministic animation for realistic dice roll behavior.

#pragma once
#include "lvgl.h"
#include <math.h>
#include <stdint.h>
#include <algorithm>
#include "esp_system.h"   // esp_random()

#if LV_COLOR_DEPTH != 16
#error "page_dice.h expects LV_COLOR_DEPTH=16 (RGB565)."
#endif

namespace dice_sim {

static constexpr float kPI = 3.14159265358979323846f;

// ---------------- frame buffer helpers ----------------
static uint16_t* g_fb=nullptr; static int g_fbW=0,g_fbH=0;

static inline bool ensure_canvas(lv_obj_t* cv){
  auto img=(lv_img_dsc_t*)lv_canvas_get_img(cv);
  int W=lv_obj_get_width(cv), H=lv_obj_get_height(cv);
  if (W<=0||H<=0) return false;
  if (!img || img->header.w!=W || img->header.h!=H || img->header.cf!=LV_IMG_CF_TRUE_COLOR){
    size_t sz=(size_t)W*(size_t)H*2;
    void* buf=lv_mem_alloc(sz); if(!buf) return false;
    if (img && img->data) lv_mem_free((void*)img->data);
    static lv_img_dsc_t desc;
    desc.header.always_zero=0; desc.header.w=W; desc.header.h=H; desc.header.cf=LV_IMG_CF_TRUE_COLOR;
    desc.data_size=sz; desc.data=(const uint8_t*)buf;
    lv_canvas_set_buffer(cv,&desc,W,H,LV_IMG_CF_TRUE_COLOR);
  }
  return true;
}
static inline bool begin_frame(lv_obj_t* cv){
  if(!ensure_canvas(cv)) return false;
  auto img=(lv_img_dsc_t*)lv_canvas_get_img(cv);
  g_fb=(uint16_t*)img->data; g_fbW=img->header.w; g_fbH=img->header.h;
  return g_fb && g_fbW>0 && g_fbH>0;
}
static inline void end_frame(lv_obj_t* cv){ lv_obj_invalidate(cv); }
static inline void putpx(int x,int y,uint16_t c){
  if((unsigned)x>=(unsigned)g_fbW || (unsigned)y>=(unsigned)g_fbH) return;
  g_fb[y*g_fbW+x]=c;
}

// ---------------- colors ----------------
static inline uint16_t rgb565(uint8_t r,uint8_t g,uint8_t b){ return lv_color_make(r,g,b).full; }
static inline uint16_t mix(uint16_t a,uint16_t b,uint8_t t){ lv_color_t ca{.full=a}, cb{.full=b}; return lv_color_mix(ca,cb,t).full; }
static inline uint16_t shade(uint16_t base,float l){
  l=l<0?0:(l>1?1:l);
  int r=((base>>11)&31)*255/31, g=((base>>5)&63)*255/63, b=(base&31)*255/31;
  r=(int)lroundf(r*l); g=(int)lroundf(g*l); b=(int)lroundf(b*l);
  return rgb565((uint8_t)std::min(255,r),(uint8_t)std::min(255,g),(uint8_t)std::min(255,b));
}
static const uint16_t C_BG=lv_color_hex(0x18181C).full;
static const uint16_t C_FLOOR_FAR=lv_color_hex(0x1C1F25).full;
static const uint16_t C_FLOOR_NEAR=lv_color_hex(0x262A31).full;
static const uint16_t C_GRID=lv_color_hex(0x2E3340).full;
static const uint16_t C_IVORY=rgb565(248,246,240), C_PIP=rgb565(18,18,22), C_EDGE=rgb565(36,36,42);

// ---------------- cube geometry & raster ----------------
struct FaceAxes{ float nx,ny,nz, ux,uy,uz, vx,vy,vz; };
// 0:+Z,1:+X,2:-X,3:-Y,4:+Y,5:-Z  (values: 1..6 on faces)
static const FaceAxes kAxes[6]={
  {0,0, 1, 1,0,0, 0,1,0},
  {1,0, 0, 0,0,-1,0,1,0},
  {-1,0,0, 0,0, 1, 0,1,0},
  {0,-1,0, 1,0,0, 0,0,1},
  {0, 1,0,-1,0,0, 0,0,1},
  {0,0,-1, 1,0,0, 0,-1,0},
};
template<typename Emit> static inline void pips_uv(int v, Emit emit){
  auto dot=[&](float u,float w){ emit(u,w); };
  switch(std::max(1,std::min(6,v))){
    case 1: dot(0,0); break;
    case 2: dot(-0.6f,-0.6f); dot(0.6f,0.6f); break;
    case 3: dot(-0.6f,-0.6f); dot(0,0); dot(0.6f,0.6f); break;
    case 4: dot(-0.6f,-0.6f); dot(0.6f,-0.6f); dot(-0.6f,0.6f); dot(0.6f,0.6f); break;
    case 5: dot(-0.6f,-0.6f); dot(0.6f,-0.6f); dot(0,0); dot(-0.6f,0.6f); dot(0.6f,0.6f); break;
    default: for(float vv=-0.6f;vv<=0.61f;vv+=0.6f){ dot(-0.6f,vv); dot(0.6f,vv); }
  }
}
struct V3{ float x,y,z,u,v; };
static inline void Rx(float d,float M[9]){ float r=d*0.0174532925f,c=cosf(r),s=sinf(r); M[0]=1;M[1]=0;M[2]=0;M[3]=0;M[4]=c;M[5]=-s;M[6]=0;M[7]=s;M[8]=c; }
static inline void Ry(float d,float M[9]){ float r=d*0.0174532925f,c=cosf(r),s=sinf(r); M[0]=c;M[1]=0;M[2]=s;M[3]=0;M[4]=1;M[5]=0;M[6]=-s;M[7]=0;M[8]=c; }
static inline void Rz(float d,float M[9]){ float r=d*0.0174532925f,c=cosf(r),s=sinf(r); M[0]=c;M[1]=-s;M[2]=0;M[3]=s;M[4]=c;M[5]=0;M[6]=0;M[7]=0;M[8]=1; }
static inline void mat_mul3(const float A[9],const float B[9],float O[9]){
  O[0]=A[0]*B[0]+A[1]*B[3]+A[2]*B[6]; O[1]=A[0]*B[1]+A[1]*B[4]+A[2]*B[7]; O[2]=A[0]*B[2]+A[1]*B[5]+A[2]*B[8];
  O[3]=A[3]*B[0]+A[4]*B[3]+A[5]*B[6]; O[4]=A[3]*B[1]+A[4]*B[4]+A[5]*B[7];
  O[5]=A[3]*B[2]+A[4]*B[5]+A[5]*B[8]; O[6]=A[6]*B[0]+A[7]*B[3]+A[8]*B[6];
  O[7]=A[6]*B[1]+A[7]*B[4]+A[8]*B[7]; O[8]=A[6]*B[2]+A[7]*B[5]+A[8]*B[8];
}
static inline void make_pre_rotation_for_top(int top,float S[9]){
  S[0]=1;S[1]=0;S[2]=0;S[3]=0;S[4]=1;S[5]=0;S[6]=0;S[7]=0;S[8]=1;
  switch(top){ case 6: Rx(180,S); break; case 2: Rx(-90,S); break; case 5: Rx(90,S); break; case 3: Ry(-90,S); break; case 4: Ry(90,S); break; default: break; }
}
static inline void draw_edge_line(float* zbuf,int W,int H,V3 a,V3 b,uint16_t col,float bias=1e-3f){
  float dx=b.x-a.x, dy=b.y-a.y; int steps=(int)fmaxf(fabsf(dx),fabsf(dy)); if(steps<=0) steps=1;
  float sx=dx/steps, sy=dy/steps, sz=(b.z-a.z)/steps; float x=a.x,y=a.y,z=a.z;
  for(int i=0;i<=steps;i++){ int ix=(int)lroundf(x), iy=(int)lroundf(y);
    if((unsigned)ix<(unsigned)W && (unsigned)iy<(unsigned)H){ int idx=iy*W+ix; float dz=z+bias; if(dz>zbuf[idx]){ zbuf[idx]=dz; putpx(ix,iy,col);} }
    x+=sx; y+=sy; z+=sz;
  }
}
static inline void raster_tri(float* zbuf,int W,int H,V3 a,V3 b,V3 c,uint16_t col,bool draw_pips,int face_val){
  auto edge=[](float x0,float y0,float x1,float y1,float x,float y){ return (x-x0)*(y1-y0)-(y-y0)*(x1-x0); };
  float minx=floorf(std::min({a.x,b.x,c.x})), maxx=ceilf(std::max({a.x,b.x,c.x}));
  float miny=floorf(std::min({a.y,b.y,c.y})), maxy=ceilf(std::max({a.y,b.y,c.y}));
  int x0=std::max(0,(int)minx), x1=std::min(W-1,(int)maxx); int y0=std::max(0,(int)miny), y1=std::min(H-1,(int)maxy);
  float area=edge(a.x,a.y,b.x,b.y,c.x,c.y); if(area==0) return;
  float pu[7],pv[7]; int pc=0; if(draw_pips){ pips_uv(face_val,[&](float uu,float vv){ if(pc<7){ pu[pc]=uu; pv[pc]=vv; ++pc; } }); }
  const float r2=0.20f*0.20f;
  for(int y=y0;y<=y1;y++){ float py=y+0.5f; for(int x=x0;x<=x1;x++){ float px=x+0.5f;
    float w0=edge(b.x,b.y,c.x,c.y,px,py), w1=edge(c.x,c.y,a.x,a.y,px,py), w2=edge(a.x,a.y,b.x,b.y,px,py);
    if(!((w0>=0&&w1>=0&&w2>=0)||(w0<=0&&w1<=0&&w2<=0))) continue; w0/=area; w1/=area; w2/=area;
    float z=a.z*w0+b.z*w1+c.z*w2; int idx=y*W+x; if(z>zbuf[idx]){ zbuf[idx]=z; putpx(x,y,col);
      if(draw_pips && pc){ float u=a.u*w0+b.u*w1+c.u*w2, v=a.v*w0+b.v*w1+c.v*w2;
        for(int i=0;i<pc;i++){ float du=u-pu[i], dv=v-pv[i]; if(du*du+dv*dv<r2){ putpx(x,y,C_PIP); break; } } }
    }
  } }
}

// ---------------- floor / room ----------------
static inline void clear_bg(int W,int H,uint16_t col){ for(int i=0;i<W*H;i++) g_fb[i]=col; }
static inline void draw_floor_room(int W,int H,int vp_x,int y_h){
  if(y_h<0) y_h=0; if(y_h>=H) y_h=H-1;
  for(int y=y_h;y<H;++y){
    float t=(float)(y-y_h)/(float)std::max(1,H-y_h-1);
    uint8_t mixv=(uint8_t)lroundf(t*255.0f);
    uint16_t row_col=mix(C_FLOOR_FAR,C_FLOOR_NEAR,mixv);
    for(int x=0;x<W;++x){
      float nx=fabsf((x-(W*0.5f))/(W*0.5f));
      uint8_t vfade=(uint8_t)lroundf(std::min(1.0f,nx*0.6f)*255.0f);
      g_fb[y*W+x]=mix(row_col,C_BG,vfade);
    }
  }
  const int rails=6;
  for(int i=0;i<=rails;++i){
    int xb=(int)lroundf(((float)i/rails)*(W-1));
    int x0=vp_x,y0=y_h,x1=xb,y1=H-1;
    int dx=abs(x1-x0), sx=x0<x1?1:-1, dy=-abs(y1-y0), sy=y0<y1?1:-1, err=dx+dy;
    for(;;){
      int idx=y0*W+x0; g_fb[idx]=mix(g_fb[idx],C_GRID,28);
      if(x0==x1 && y0==y1) break;
      int e2=2*err; if(e2>=dy){ err+=dy; x0+=sx; } if(e2<=dx){ err+=dx; y0+=sy; }
      if((unsigned)x0>=(unsigned)W || (unsigned)y0>=(unsigned)H) break;
    }
  }
  const int rungs=6;
  for(int i=1;i<=rungs;++i){
    float u=(float)i/(float)(rungs+1), eased=u*u; int y=y_h+(int)lroundf(eased*(H-1-y_h));
    if(y<0||y>=H) continue; for(int x=0;x<W;++x){ int idx=y*W+x; g_fb[idx]=mix(g_fb[idx],C_GRID,22); }
  }
}

// ---------------- draw cube ----------------
static const float VDX=-0.7071f, VDY=-0.35f, VDZ=1.10f;
struct Params{
  lv_obj_t* cv; int cx_px,cy_px; int top_face; float size_scale;
  float ax_deg,ay_deg,az_deg; float cam_lerp; int pan_y_px;
};
static inline void draw(const Params& p){
  if(!begin_frame(p.cv)) return; const int W=g_fbW,H=g_fbH;
  clear_bg(W,H,C_BG);

  const float Smin=(float)std::min(W,H);
  const float half_base=Smin/3.0f < 10.0f ? 10.0f : Smin/3.0f;
  const float half=half_base*(p.size_scale<=0?1.0f:p.size_scale);

  int vp_x = W/2;
  int base_y_h = H/2 + (int)lroundf(half*0.25f);
  int y_h = base_y_h - p.pan_y_px;
  draw_floor_room(W,H,vp_x,y_h);

  static float* zbuf=nullptr; static int zcap=0; int need=W*H;
  if(need>zcap){ float* nb=(float*)lv_mem_realloc(zbuf,need*sizeof(float)); if(!nb){ end_frame(p.cv); return; } zbuf=nb; zcap=need; }
  for(int i=0;i<need;i++) zbuf[i]=-1e9f;

  float RX[9],RY[9],RZ[9],Rxy[9],Reul[9]; Rx(p.ax_deg,RX); Ry(p.ay_deg,RY); Rz(p.az_deg,RZ); mat_mul3(RY,RX,Rxy); mat_mul3(RZ,Rxy,Reul);
  float Rcam[9]; Rx(-55.0f*(p.cam_lerp<0?0:(p.cam_lerp>1?1:p.cam_lerp)),Rcam);
  float S[9]; make_pre_rotation_for_top(p.top_face,S);
  float Robj[9],R[9]; mat_mul3(Reul,S,Robj); mat_mul3(Rcam,Robj,R);

  auto rot=[&](float x,float y,float z)->V3{
    return {R[0]*x+R[1]*y+R[2]*z, R[3]*x+R[4]*y+R[5]*z, R[6]*x+R[7]*y+R[8]*z, 0,0};
  };

  struct F{ int idx; float cx,cy,cz,zc; bool pips; int val; } faces[6]={
    {0,0,0,half,0,true,1},{1,half,0,0,0,true,3},{2,-half,0,0,0,true,4},
    {3,0,-half,0,0,true,2},{4,0,half,0,0,true,5},{5,0,0,-half,0,true,6}
  };
  for(auto& f:faces){ V3 C=rot(f.cx,f.cy,f.cz); f.zc=C.z; }
  std::sort(std::begin(faces),std::end(faces),[](const F&a,const F&b){ return a.zc<b.zc; });

  auto draw_face = [&](const FaceAxes& ax,float tx,float ty,float tz,bool with_pips,int face_val){
    V3 U=rot(ax.ux,ax.uy,ax.uz), V=rot(ax.vx,ax.vy,ax.vz), C=rot(tx,ty,tz);
    float q[4][2]={{-half,-half},{half,-half},{half,half},{-half,half}}; V3 v[4];
    for(int i=0;i<4;i++){
      float uu=q[i][0], vv=q[i][1];
      float x=C.x+uu*U.x+vv*V.x, y=C.y+uu*U.y+vv*V.y, z=C.z+uu*U.z+vv*V.z;
      float X=(x-y)*0.65f + (float)(W/2 + p.cx_px);
      float Y=(x+y)*0.35f - z*1.10f + (float)(H/2 + p.cy_px - p.pan_y_px);
      float D=x*VDX + y*VDY + z*VDZ; v[i]={X,Y,D, uu/half, vv/half};
    }
    V3 N=rot(ax.nx,ax.ny,ax.nz); float L=std::max(0.0f, N.x*(-0.4f)+N.y*(-0.3f)+N.z*(0.85f))*0.6f+0.4f;
    uint16_t col=shade(C_IVORY,L);
    raster_tri(zbuf,W,H,v[0],v[1],v[2],col,with_pips,face_val);
    raster_tri(zbuf,W,H,v[0],v[2],v[3],col,with_pips,face_val);
    draw_edge_line(zbuf,W,H,v[0],v[1],C_EDGE); draw_edge_line(zbuf,W,H,v[1],v[2],C_EDGE);
    draw_edge_line(zbuf,W,H,v[2],v[3],C_EDGE); draw_edge_line(zbuf,W,H,v[3],v[0],C_EDGE);
  };
  for(auto& f:faces){ const auto& ax=kAxes[f.idx]; draw_face(ax,f.cx,f.cy,f.cz,true,f.val); }

  end_frame(p.cv);
}

// ---------------- utils ----------------
static float wrap_deg(float a){ while(a<=-180.f) a+=360.f; while(a>180.f) a-=360.f; return a; }
static float nearest90(float a){ return roundf(a/90.0f)*90.0f; }
static inline float len2(float x,float y){ return sqrtf(x*x+y*y); }


// ---------------- animation-based simulation ----------------
struct Sim{
  lv_obj_t* cv=nullptr; lv_timer_t* tmr=nullptr; int W=0,H=0;

  // Render-space state driven by animation
  float px=0,py=0;               // center offset in px
  float ax=0,ay=0,az=0;          // Euler angles (deg)

  int   top_face=1;
  float size_scale=0.60f;

  // Camera: fixed during motion, reveal at end only
  const float CAM_BASE=0.15f;
  float cam=CAM_BASE; bool cam_transition=false; int cam_ticks=0;

  enum Phase{ Rolling, Settling, Locked } phase=Locked;
  bool  active=false;

  // ---------- Animation state ----------
  float anim_time=0.0f;          // Current animation time (seconds)
  float anim_duration=2.5f;      // Total animation duration

  // 3D position and velocity
  float pos_x=0, pos_y=0, pos_z=0;     // 3D position
  float vel_x=0, vel_y=0, vel_z=0;     // 3D velocity
  float start_ax=0, start_ay=0, start_az=0;  // Starting angles
  float target_ax=0, target_ay=0, target_az=0; // Target final angles

  int   bounce_count=0;          // Number of bounces so far
  float last_bounce_time=0.0f;   // Time of last bounce

  // 3D floor definition - adjusted to match perspective rendering
  // The visual floor in the perspective view corresponds to this Z level
  static constexpr float FLOOR_Z = -25.0f;

  static uint32_t urand(){ return esp_random(); }
  static float frand(float a,float b){ uint32_t r=urand(); float t=(r&0xFFFFFF)/float(0x1000000); return a+(b-a)*t; }

  // Easing functions for smooth animation
  static float ease_out_bounce(float t) {
    if (t < 1.0f / 2.75f) {
      return 7.5625f * t * t;
    } else if (t < 2.0f / 2.75f) {
      t -= 1.5f / 2.75f;
      return 7.5625f * t * t + 0.75f;
    } else if (t < 2.5f / 2.75f) {
      t -= 2.25f / 2.75f;
      return 7.5625f * t * t + 0.9375f;
    } else {
      t -= 2.625f / 2.75f;
      return 7.5625f * t * t + 0.984375f;
    }
  }

  static float ease_out_cubic(float t) {
    t = 1.0f - t;
    return 1.0f - t * t * t;
  }

  // Calculate target angles for a given face to be on top
  void calculate_target_angles_for_face(int face, float& target_ax, float& target_ay) {
    switch(face) {
      case 1: target_ax = 0;   target_ay = 0;   break;  // +Z (1 pip)
      case 2: target_ax = 90;  target_ay = 0;   break;  // -Y (2 pips)
      case 3: target_ax = 0;   target_ay = 90;  break;  // +X (3 pips)
      case 4: target_ax = 0;   target_ay = -90; break;  // -X (4 pips)
      case 5: target_ax = -90; target_ay = 0;   break;  // +Y (5 pips)
      case 6: target_ax = 180; target_ay = 0;   break;  // -Z (6 pips)
      default: target_ax = 0; target_ay = 0; break;
    }
  }

  // 3D vector and cube math
  struct Vec3 { float x, y, z; };

  // Calculate all 8 corners of the cube in 3D space
  void calculate_cube_corners(float cx, float cy, float cz, float ax_deg, float ay_deg, float az_deg, Vec3 corners[8]) {
    const float half = 20.0f;  // Half cube size in 3D units

    // Create rotation matrices
    float RX[9], RY[9], RZ[9], Rxy[9], R[9];
    Rx(ax_deg, RX); Ry(ay_deg, RY); Rz(az_deg, RZ);
    mat_mul3(RY, RX, Rxy); mat_mul3(RZ, Rxy, R);

    // 8 corners of a cube centered at origin
    Vec3 local_corners[8] = {
      {-half, -half, -half}, {+half, -half, -half}, {+half, +half, -half}, {-half, +half, -half},
      {-half, -half, +half}, {+half, -half, +half}, {+half, +half, +half}, {-half, +half, +half}
    };

    // Transform corners by rotation and position
    for(int i = 0; i < 8; i++) {
      float lx = local_corners[i].x, ly = local_corners[i].y, lz = local_corners[i].z;
      corners[i].x = cx + R[0]*lx + R[1]*ly + R[2]*lz;
      corners[i].y = cy + R[3]*lx + R[4]*ly + R[5]*lz;
      corners[i].z = cz + R[6]*lx + R[7]*ly + R[8]*lz;
    }
  }

  // Find the lowest Z value among all cube corners
  float get_lowest_corner_z(float cx, float cy, float cz, float ax_deg, float ay_deg, float az_deg) {
    Vec3 corners[8];
    calculate_cube_corners(cx, cy, cz, ax_deg, ay_deg, az_deg, corners);

    float min_z = corners[0].z;
    for(int i = 1; i < 8; i++) {
      if(corners[i].z < min_z) min_z = corners[i].z;
    }
    return min_z;
  }

  // Check if cube is "flat" on the floor (bottom face parallel to floor)
  bool is_flat_on_floor(float cx, float cy, float cz, float ax_deg, float ay_deg, float az_deg) {
    Vec3 corners[8];
    calculate_cube_corners(cx, cy, cz, ax_deg, ay_deg, az_deg, corners);

    // Find the 4 lowest corners (should form bottom face)
    float min_z = get_lowest_corner_z(cx, cy, cz, ax_deg, ay_deg, az_deg);
    const float tolerance = 2.0f;  // Increased tolerance for settling detection

    int low_corners = 0;
    for(int i = 0; i < 8; i++) {
      if(fabsf(corners[i].z - min_z) < tolerance) {
        low_corners++;
      }
    }

    // Should have exactly 4 corners at the same lowest level (within tolerance)
    return (low_corners == 4) && (min_z <= FLOOR_Z + 5.0f);
  }

  // Check if dice is in a reasonably stable orientation (not balancing on edge/corner)
  bool is_stable_orientation(float cx, float cy, float cz, float ax_deg, float ay_deg, float az_deg) {
    Vec3 corners[8];
    calculate_cube_corners(cx, cy, cz, ax_deg, ay_deg, az_deg, corners);

    // Find corners near the lowest level
    float min_z = get_lowest_corner_z(cx, cy, cz, ax_deg, ay_deg, az_deg);
    const float stability_tolerance = 3.0f;

    int low_corners = 0;
    for(int i = 0; i < 8; i++) {
      if(fabsf(corners[i].z - min_z) < stability_tolerance) {
        low_corners++;
      }
    }

    bool stable = (low_corners >= 3);
    ESP_LOGV("DICE", "Stability check: min_z=%.1f, low_corners=%d/8, stable=%d",
             min_z, low_corners, stable ? 1 : 0);

    // Stable if 3+ corners are near the ground (not balancing on 1-2 corners)
    return stable;
  }

  void begin(lv_obj_t* canvas){
    cv=canvas; ensure_canvas(cv);
    auto img=(lv_img_dsc_t*)lv_canvas_get_img(cv); W=img?img->header.w:0; H=img?img->header.h:0;
    if(!tmr){ tmr=lv_timer_create([](lv_timer_t* t){ ((Sim*)t->user_data)->tick(); },33,this); } // ~30 Hz
  }

  void end(){
    if(tmr){ lv_timer_del(tmr); tmr=nullptr; }
    cv=nullptr; active=false; cam_transition=false; phase=Locked;
  }

  void roll(){
    if(!cv) return;

    // Choose target top face (1..6)
    top_face = (int)frand(1.0f, 7.0f);
    if(top_face < 1) top_face = 1;
    if(top_face > 6) top_face = 6;

    // Set up animation parameters
    anim_time = 0.0f;
    anim_duration = frand(2.0f, 3.0f);

    // 3D starting position - choose coordinates that project to visible area
    // Target landing should project to center area of screen
    float target_3d_x = frand(-30.0f, 30.0f);  // Smaller range for better visibility
    float target_3d_y = frand(-20.0f, 20.0f);

    // Start position offset from target
    pos_x = target_3d_x + frand(-20.0f, 20.0f);
    pos_y = target_3d_y + frand(-15.0f, 15.0f);
    pos_z = 60.0f;  // Start above floor but not too high

    // 3D initial velocity toward target area
    vel_x = (target_3d_x - pos_x) * 0.8f + frand(-15.0f, 15.0f);
    vel_y = (target_3d_y - pos_y) * 0.8f + frand(-10.0f, 10.0f);
    vel_z = frand(-40.0f, -15.0f);  // Downward initial velocity

    // Random starting angles for tumbling
    start_ax = frand(0, 360);
    start_ay = frand(0, 360);
    start_az = frand(0, 360);

    // Calculate target angles to show the chosen face (when flat on floor)
    calculate_target_angles_for_face(top_face, target_ax, target_ay);
    target_az = frand(-15, 15);

    // Initialize current angles
    ax = start_ax;
    ay = start_ay;
    az = start_az;

    // Update 2D position for rendering (project 3D to 2D using isometric projection)
    px = (pos_x - pos_y) * 0.65f;
    py = (pos_x + pos_y) * 0.35f - pos_z * 1.10f;

    // Reset state
    bounce_count = 0;
    last_bounce_time = 0.0f;
    active = true;
    cam_transition = false;
    cam = CAM_BASE;
    cam_ticks = 0;
    phase = Rolling;
  }

  void start_cam_transition(){ cam_transition=true; cam_ticks=0; }
  void step_cam_transition(){
    float t=std::min(1.0f, cam_ticks/9.0f);
    cam += (1.0f-cam)*(0.25f+0.55f*t);
    cam_ticks++; if(t>=1.0f) cam_transition=false;
  }

  void animation_step(){
    const float dt = 1.0f/30.0f;  // 30 Hz step
    anim_time += dt;

    if(phase == Rolling) {
      // 3D physics simulation
      const float gravity = 200.0f;
      const float damping = 0.98f;

      // Apply gravity
      vel_z -= gravity * dt;

      // Update 3D position
      pos_x += vel_x * dt;
      pos_y += vel_y * dt;
      pos_z += vel_z * dt;

      // Check for floor collision
      float lowest_z = get_lowest_corner_z(pos_x, pos_y, pos_z, ax, ay, az);

      if(lowest_z <= FLOOR_Z) {
        // Collision with floor - bounce
        bounce_count++;

        // Adjust Z position so lowest corner touches floor
        pos_z += (FLOOR_Z - lowest_z);

        // Bounce velocity
        vel_z = fabsf(vel_z) * 0.6f;  // Reduce bounce energy
        vel_x *= damping;
        vel_y *= damping;

        // Add some randomness to bounce
        vel_x += frand(-10.0f, 10.0f);
        vel_y += frand(-10.0f, 10.0f);

        // Slow down tumbling after bounces
        float tumble_reduction = powf(0.7f, bounce_count);

        // Only transition to settling if dice is in a stable orientation
        bool low_velocity = (fabsf(vel_x) + fabsf(vel_y) + fabsf(vel_z) < 25.0f);
        bool stable = is_stable_orientation(pos_x, pos_y, pos_z, ax, ay, az);
        float total_vel = fabsf(vel_x) + fabsf(vel_y) + fabsf(vel_z);

        ESP_LOGI("DICE", "Rolling: bounce=%d, vel=%.1f, stable=%d, pos=(%.1f,%.1f,%.1f)",
                 bounce_count, total_vel, stable ? 1 : 0, pos_x, pos_y, pos_z);

        if((bounce_count >= 3 && low_velocity && stable) ||
           (bounce_count >= 5)) {  // Force settling after many bounces
          ESP_LOGI("DICE", "TRANSITION TO SETTLING: bounce=%d, vel=%.1f, stable=%d",
                   bounce_count, total_vel, stable ? 1 : 0);
          phase = Settling;
          vel_x *= 0.7f;
          vel_y *= 0.7f;
          vel_z *= 0.7f;
        }
      }

      // Bounds checking - keep dice in reasonable area
      const float max_3d_range = 40.0f;
      if(fabsf(pos_x) > max_3d_range || fabsf(pos_y) > max_3d_range) {
        // Gently pull back toward center
        vel_x -= pos_x * 0.5f * dt;
        vel_y -= pos_y * 0.5f * dt;
      }

      // Tumbling during flight (less chaotic than before)
      float tumble_speed = 180.0f * powf(0.8f, bounce_count);  // Slower tumbling
      ax += vel_x * 0.5f * dt;  // Tumbling related to velocity
      ay += vel_y * 0.5f * dt;
      az += tumble_speed * dt;

    } else if(phase == Settling) {
      // Settling phase - smooth transition to flat orientation

      // Heavy damping to stop motion quickly
      vel_x *= 0.8f;
      vel_y *= 0.8f;
      vel_z *= 0.9f;

      // Only apply physics if still moving significantly
      if(fabsf(vel_x) + fabsf(vel_y) + fabsf(vel_z) > 2.0f) {
        pos_x += vel_x * dt;
        pos_y += vel_y * dt;
        pos_z += vel_z * dt;
      } else {
        // Stop physics motion, focus on orientation
        vel_x = vel_y = vel_z = 0;
      }

      // Constrain to visible area
      const float max_3d_range = 25.0f;
      if(fabsf(pos_x) > max_3d_range) {
        pos_x = (pos_x > 0) ? max_3d_range : -max_3d_range;
      }
      if(fabsf(pos_y) > max_3d_range) {
        pos_y = (pos_y > 0) ? max_3d_range : -max_3d_range;
      }

      // Smoothly settle onto floor
      float lowest_z = get_lowest_corner_z(pos_x, pos_y, pos_z, ax, ay, az);
      float floor_error = lowest_z - FLOOR_Z;

      if(floor_error < 0) {
        // Cube is below floor, push up
        pos_z += (-floor_error);
      } else if(floor_error > 0.5f) {
        // Cube is above floor, settle down gently
        pos_z -= floor_error * 0.3f * dt * 60.0f;  // Gentle settling
      }

      // Only guide toward target if dice is already reasonably stable
      bool settling_stable = is_stable_orientation(pos_x, pos_y, pos_z, ax, ay, az);
      float settle_vel = fabsf(vel_x) + fabsf(vel_y) + fabsf(vel_z);

      if(settling_stable) {
        // Smooth orientation toward target (no jerky steps)
        float angle_diff_x = target_ax - ax;
        float angle_diff_y = target_ay - ay;
        float angle_diff_z = target_az - az;

        // Handle angle wrapping for shortest path
        while(angle_diff_x > 180) angle_diff_x -= 360;
        while(angle_diff_x < -180) angle_diff_x += 360;
        while(angle_diff_y > 180) angle_diff_y -= 360;
        while(angle_diff_y < -180) angle_diff_y += 360;
        while(angle_diff_z > 180) angle_diff_z -= 360;
        while(angle_diff_z < -180) angle_diff_z += 360;

        float total_angle_diff = fabsf(angle_diff_x) + fabsf(angle_diff_y) + fabsf(angle_diff_z);
        ESP_LOGI("DICE", "Settling(STABLE): vel=%.1f, angle_diff=%.1f, target=(%d,%d,%d), current=(%.1f,%.1f,%.1f)",
                 settle_vel, total_angle_diff, (int)target_ax, (int)target_ay, (int)target_az, ax, ay, az);

        // Smooth interpolation (no sudden jumps)
        const float smooth_rate = 2.0f;  // Slower, more gentle
        ax += angle_diff_x * smooth_rate * dt;
        ay += angle_diff_y * smooth_rate * dt;
        az += angle_diff_z * smooth_rate * dt;
      } else {
        ESP_LOGI("DICE", "Settling(UNSTABLE): vel=%.1f, letting physics settle naturally", settle_vel);
        // If not stable, let physics settle more naturally with minimal tumbling
        float natural_damping = 0.95f;
        ax *= natural_damping;
        ay *= natural_damping;
        az *= natural_damping;
      }

      // Check if settled - use proper flat detection
      bool stopped = (fabsf(vel_x) + fabsf(vel_y) + fabsf(vel_z) < 1.0f);

      // Calculate angle differences for checking
      float check_angle_diff_x = target_ax - ax;
      float check_angle_diff_y = target_ay - ay;
      float check_angle_diff_z = target_az - az;
      while(check_angle_diff_x > 180) check_angle_diff_x -= 360;
      while(check_angle_diff_x < -180) check_angle_diff_x += 360;
      while(check_angle_diff_y > 180) check_angle_diff_y -= 360;
      while(check_angle_diff_y < -180) check_angle_diff_y += 360;
      while(check_angle_diff_z > 180) check_angle_diff_z -= 360;
      while(check_angle_diff_z < -180) check_angle_diff_z += 360;

      bool correct_angles = (fabsf(check_angle_diff_x) + fabsf(check_angle_diff_y) + fabsf(check_angle_diff_z) < 5.0f);
      bool truly_flat = is_flat_on_floor(pos_x, pos_y, pos_z, ax, ay, az);
      float final_angle_diff = fabsf(check_angle_diff_x) + fabsf(check_angle_diff_y) + fabsf(check_angle_diff_z);

      ESP_LOGI("DICE", "Final check: stopped=%d, correct_angles=%d(%.1f), truly_flat=%d",
               stopped ? 1 : 0, correct_angles ? 1 : 0, final_angle_diff, truly_flat ? 1 : 0);

      if(stopped && correct_angles && truly_flat) {
        ESP_LOGI("DICE", "LOCKING: face=%d, final_pos=(%.1f,%.1f,%.1f), final_angles=(%.1f,%.1f,%.1f)",
                 top_face, pos_x, pos_y, pos_z, ax, ay, az);

        // Lock in final flat state
        ax = target_ax;
        ay = target_ay;
        az = target_az;
        vel_x = vel_y = vel_z = 0;

        // Force cube exactly on floor with exact orientation
        lowest_z = get_lowest_corner_z(pos_x, pos_y, pos_z, ax, ay, az);
        pos_z += (FLOOR_Z - lowest_z);

        phase = Locked;
        active = false;
        if(!cam_transition) start_cam_transition();
      }
    }

    // Update 2D rendering position (project 3D to screen using isometric projection)
    // This matches the projection used in the cube rendering:
    // X = (x-y)*0.65f + offset, Y = (x+y)*0.35f - z*1.10f + offset
    px = (pos_x - pos_y) * 0.65f;
    py = (pos_x + pos_y) * 0.35f - pos_z * 1.10f;

    ax = wrap_deg(ax);
    ay = wrap_deg(ay);
    az = wrap_deg(az);
  }

  void render(){
    if(!cv) return;
    const float Smin=(float)std::min(W,H);
    const float half_base=std::max(10.0f,Smin/3.0f);
    const float half=half_base*size_scale;
    int cam_pan = (cam<=CAM_BASE+1e-3f)? 0 : (int)lroundf((cam-CAM_BASE)*half*0.9f);

    Params p; p.cv=cv; p.cx_px=(int)lroundf(px); p.cy_px=(int)lroundf(py);
    p.top_face=top_face; p.size_scale=size_scale;
    p.ax_deg=ax; p.ay_deg=ay; p.az_deg=az; p.cam_lerp=cam; p.pan_y_px=cam_pan;
    draw(p);
  }

  void tick(){
    if(active){ animation_step(); render(); }
    else if(cam_transition){ step_cam_transition(); render(); }
    else { render(); }
  }
};

static Sim G;

// ---------------- public wrappers ----------------
static inline void dice_on_load(lv_obj_t* canvas){ G.begin(canvas); G.render(); }
static inline void dice_on_unload(lv_obj_t* /*canvas*/){ G.end(); }
static inline void dice_roll(){ G.roll(); }
static inline int  dice_current_target_top(){ return G.top_face; }

} // namespace dice_sim

#ifdef __cplusplus
extern "C++" {
#endif
static inline void dice_on_load(lv_obj_t* canvas){ dice_sim::dice_on_load(canvas); }
static inline void dice_on_unload(lv_obj_t* canvas){ dice_sim::dice_on_unload(canvas); }
static inline void dice_roll(){ dice_sim::dice_roll(); }
static inline int  dice_current_target_top(){ return dice_sim::dice_current_target_top(); }
#ifdef __cplusplus
}
#endif
