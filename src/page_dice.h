// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// src/page_dice.h — D6 toss using shared Chipmunk2D cpSpace (from chipmunk2d component)
// Renders a 3D-looking die in LVGL, uses Chipmunk for motion + yaw, tiny end-only snap to land flat.

#pragma once
#include "lvgl.h"
#include <math.h>
#include <stdint.h>
#include <algorithm>
#include "esp_system.h"   // esp_random()

// NOTE: Do NOT wrap in extern "C" — Chipmunk provides C++ operator overloads for cpVect.
#include <chipmunk/chipmunk.h>

// Borrow the cpSpace the component owns/steps.
extern "C" cpSpace* chipmunk2d_get_space();

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

// ---------------- simulation (shared cpSpace) ----------------
struct Sim{
  lv_obj_t* cv=nullptr; lv_timer_t* tmr=nullptr; int W=0,H=0;

  // Render-space state driven by physics
  float px=0,py=0;               // center offset in px (Chipmunk world units)
  float ax=0,ay=0,az=0;          // Euler angles (deg)
  float wz_vis=0;                // visual Z spin (deg/s)

  int   top_face=1;
  float size_scale=0.60f;

  // Camera: fixed during motion, reveal at end only
  const float CAM_BASE=0.15f;
  float cam=CAM_BASE; bool cam_transition=false; int cam_ticks=0;

  enum Phase{ Air, Sliding, Settle, Locked } phase=Air;
  int   settle_ticks=0;
  bool  active=false;

  // ---------- Chipmunk pointers (component owns the space) ----------
  cpSpace* space=nullptr;        // borrowed (do not free)
  cpBody*  body=nullptr;         // we own
  cpShape* box=nullptr;          // we own
  cpShape* floor=nullptr;        // we own
  cpShape* leftw=nullptr;        // we own
  cpShape* rightw=nullptr;       // we own
  cpShape* roof=nullptr;         // we own

  float box_size_px=40.0f;
  float floorY=0.0f, leftX=0.0f, rightX=0.0f, topY=0.0f;

  static uint32_t urand(){ return esp_random(); }
  static float frand(float a,float b){ uint32_t r=urand(); float t=(r&0xFFFFFF)/float(0x1000000); return a+(b-a)*t; }

  float margin() const{
    float smin=(float)std::min(W,H); float half_base=std::max(10.0f,smin/3.0f);
    float half=half_base*size_scale; return half*1.25f;
  }

  void free_ours_(){
    if(!space) return;
    if(box){    cpSpaceRemoveShape(space, box);    cpShapeFree(box);    box=nullptr; }
    if(body){   cpSpaceRemoveBody(space, body);    cpBodyFree(body);    body=nullptr; }
    if(floor){  cpSpaceRemoveShape(space, floor);  cpShapeFree(floor);  floor=nullptr; }
    if(leftw){  cpSpaceRemoveShape(space, leftw);  cpShapeFree(leftw);  leftw=nullptr; }
    if(rightw){ cpSpaceRemoveShape(space, rightw); cpShapeFree(rightw); rightw=nullptr; }
    if(roof){   cpSpaceRemoveShape(space, roof);   cpShapeFree(roof);   roof=nullptr; }
  }

  void build_into_shared_space_(){
    free_ours_();
    space = chipmunk2d_get_space();
    if(!space || W<=0 || H<=0) return;

    float m=margin();
    leftX = -(W*0.5f - m);
    rightX= +(W*0.5f - m);
    topY  = -(H*0.5f - m);
    floorY= +(H*0.5f - m);

    cpBody* staticBody = cpSpaceGetStaticBody(space);

    // tiny edge radius aids stability; moderate elasticity for lively bounces
    const cpFloat edge_r = 1.5f;
    floor = cpSegmentShapeNew(staticBody, cpv(-10000, floorY), cpv(10000, floorY), edge_r);
    leftw = cpSegmentShapeNew(staticBody,  cpv(leftX, -10000), cpv(leftX, 10000), edge_r);
    rightw= cpSegmentShapeNew(staticBody,  cpv(rightX,-10000), cpv(rightX,10000), edge_r);
    roof  = cpSegmentShapeNew(staticBody,  cpv(-10000, topY),  cpv(10000, topY),  edge_r);

    for(auto s: {floor,leftw,rightw,roof}){
      cpShapeSetFriction(s, 0.70f);
      cpShapeSetElasticity(s, 0.35f);
      cpSpaceAddShape(space, s);
    }

    float Smin=(float)std::min(W,H);
    float half_base=std::max(10.0f,Smin/3.0f);
    float half=half_base*size_scale;
    box_size_px = half*2.0f;

    cpFloat mass=1.0f, moment=cpMomentForBox(mass, box_size_px, box_size_px);
    body = cpBodyNew(mass, moment);
    box  = cpBoxShapeNew(body, box_size_px, box_size_px, 0);
    cpShapeSetFriction(box, 0.74f);
    cpShapeSetElasticity(box, 0.22f);

    cpSpaceAddBody(space, body);
    cpSpaceAddShape(space, box);
  }

  void begin(lv_obj_t* canvas){
    cv=canvas; ensure_canvas(cv);
    auto img=(lv_img_dsc_t*)lv_canvas_get_img(cv); W=img?img->header.w:0; H=img?img->header.h:0;
    build_into_shared_space_();
    if(!tmr){ tmr=lv_timer_create([](lv_timer_t* t){ ((Sim*)t->user_data)->tick(); },33,this); } // ~30 Hz
  }

  void end(){
    if(tmr){ lv_timer_del(tmr); tmr=nullptr; }
    free_ours_();                        // only our shapes/bodies
    space=nullptr;                       // component still owns cpSpace
    cv=nullptr; active=false; cam_transition=false; phase=Air;
  }

  void roll(){
    if(!cv) return;
    if(!space){ build_into_shared_space_(); if(!space) return; }

    // Choose target top face now (1..6)
    top_face=(int)frand(1.0f,7.0f); if(top_face<1) top_face=1; if(top_face>6) top_face=6;

    // start near center, slightly above floor
    float x0 = frand(-W*0.15f,  W*0.15f);
    float y0 = frand(-H*0.18f, -H*0.06f);

    // decent lateral + downward velocity
    float vx = frand(-620.0f,  620.0f);
    float vy = frand(-880.0f,  -380.0f);
    float ang = frand(0, kPI*2.0f);
    float angv= frand(-20.0f, 20.0f); // rad/s

    cpBodySetPosition(body, cpv(x0, y0));
    cpBodySetVelocity(body,  cpv(vx, vy));
    cpBodySetAngle(body,     ang);
    cpBodySetAngularVelocity(body, angv);

    // wake + tiny impulses to avoid “dead” starts and encourage wall ricochets
    cpBodyActivate(body);
    cpBodyApplyImpulseAtLocalPoint(body, cpv(vx*0.03f, vy*0.03f), cpv(0,0));
    cpVect hit = cpv(frand(-box_size_px*0.45f, box_size_px*0.45f),
                     frand(-box_size_px*0.45f, box_size_px*0.45f));
    cpBodyApplyImpulseAtLocalPoint(body, cpv(frand(-220.0f,220.0f), frand(-180.0f,180.0f)), hit);

    // visuals (X/Y tumble are decorative; Z follows physics angle)
    ax=frand(0,360); ay=frand(0,360);
    az=(float)(ang * 180.0f / kPI);
    wz_vis=(float)(angv * 180.0f / kPI);

    active=true; cam_transition=false; cam=CAM_BASE; cam_ticks=0;
    settle_ticks=0; phase=Air;
  }

  void start_cam_transition(){ cam_transition=true; cam_ticks=0; }
  void step_cam_transition(){
    float t=std::min(1.0f, cam_ticks/9.0f);
    cam += (1.0f-cam)*(0.25f+0.55f*t);
    cam_ticks++; if(t>=1.0f) cam_transition=false;
  }

  bool near_floor_() const {
    cpBB bb = cpShapeCacheBB((cpShape*)box);
    return (bb.b >= floorY - 0.5f);
  }

  // ---- tiny helpers to rely on Chipmunk contact info ----
  static void _count_arb_cb(cpBody* /*b*/, cpArbiter* /*arb*/, void* data){
    int* n = (int*)data; (*n)++;
  }
  bool touching_any_surface_() const {
    int n = 0; cpBodyEachArbiter(body, _count_arb_cb, &n); return n > 0;
  }
  void snap_body_down_to_floor_() {
    cpBB bb = cpShapeCacheBB((cpShape*)box);
    float dy = floorY - bb.b;
    if (fabsf(dy) > 0.01f) {
      cpVect p = cpBodyGetPosition(body);
      cpBodySetPosition(body, cpv(p.x, p.y + dy));
    }
  }

  void physics_step_readback_(){
    // Space is stepped by the component; just read body state and drive visuals.
    cpVect p = cpBodyGetPosition(body);
    cpVect v = cpBodyGetVelocity(body);
    float ang = (float)cpBodyGetAngle(body);
    float angv= (float)cpBodyGetAngularVelocity(body);

    px = p.x; py = p.y;
    az = (float)(ang * 180.0f / kPI);
    wz_vis = (float)(angv * 180.0f / kPI);

    // a tiny visual tumble on X/Y while moving (decorative)
    const float dt_frame = 1.0f/30.0f;
    if(phase==Air || phase==Sliding){
      float tumble = std::min(1.0f, fabsf(wz_vis)/180.0f);
      ax += (wz_vis*0.12f) * dt_frame * tumble;
      ay += (wz_vis*0.08f)  * dt_frame * tumble;
      ax *= 0.992f; ay *= 0.992f;
    }

    float lin = len2(v.x, v.y);
    float ang_abs = fabsf(wz_vis);

    // --- phase transitions driven by contact + low velocities ---
    if(phase==Air){
      if(touching_any_surface_()) phase=Sliding;
    }else if(phase==Sliding){
      if(!touching_any_surface_()) {
        phase=Air;
      } else if(lin < 20.0f && ang_abs < 50.0f){
        phase=Settle; settle_ticks=0;
      }
    }else if(phase==Settle){
      // minimal proportional snap to nearest 90°
      float tx=nearest90(ax), ty=nearest90(ay);
      float ex=ax-tx, ey=ay-ty;
      ax -= 0.22f * ex;
      ay -= 0.22f * ey;

      settle_ticks++;
      bool aligned = (fabsf(ex)<1.0f && fabsf(ey)<1.0f);
      if(aligned && settle_ticks>=6 && touching_any_surface_()){
        ax=tx; ay=ty;
        cpBodySetVelocity(body, cpv(0,0));
        cpBodySetAngularVelocity(body, 0);
        if(near_floor_()) snap_body_down_to_floor_();
        phase=Locked; active=false;
        if(!cam_transition) start_cam_transition();
      }
    }

    // Safety hard stop near the floor
    if(phase!=Locked){
      if(lin < 3.0f && ang_abs < 5.0f && near_floor_()){
        ax=nearest90(ax); ay=nearest90(ay);
        cpBodySetVelocity(body, cpv(0,0));
        cpBodySetAngularVelocity(body, 0);
        snap_body_down_to_floor_();
        phase=Locked; active=false; cam=1.0f; cam_transition=false;
      }
    }

    ax=wrap_deg(ax); ay=wrap_deg(ay); az=wrap_deg(az);
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
    if(!space){ render(); return; }
    if(active){ physics_step_readback_(); render(); }
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
