#include "selfdrive/ui/paint.h"

#include <algorithm>
#include <cassert>

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#define NANOVG_GL3_IMPLEMENTATION
#define nvgCreate nvgCreateGL3
#else
#include <GLES3/gl3.h>
#define NANOVG_GLES3_IMPLEMENTATION
#define nvgCreate nvgCreateGLES3
#endif

#define NANOVG_GLES3_IMPLEMENTATION
#include <nanovg_gl.h>
#include <nanovg_gl_utils.h>

#include "selfdrive/common/timing.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"

#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/extras.h"

static void ui_draw_text(const UIState *s, float x, float y, const char *string, float size, NVGcolor color, const char *font_name) {
  nvgFontFace(s->vg, font_name);
  nvgFontSize(s->vg, size);
  nvgFillColor(s->vg, color);
  nvgText(s->vg, x, y, string, NULL);
}

static void draw_chevron(UIState *s, float x, float y, float sz, NVGcolor fillColor, NVGcolor glowColor) {
  // glow
  float g_xo = sz/5;
  float g_yo = sz/10;
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x+(sz*1.35)+g_xo, y+sz+g_yo);
  nvgLineTo(s->vg, x, y-g_xo);
  nvgLineTo(s->vg, x-(sz*1.35)-g_xo, y+sz+g_yo);
  nvgClosePath(s->vg);
  nvgFillColor(s->vg, glowColor);
  nvgFill(s->vg);

  // chevron
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x+(sz*1.25), y+sz);
  nvgLineTo(s->vg, x, y);
  nvgLineTo(s->vg, x-(sz*1.25), y+sz);
  nvgClosePath(s->vg);
  nvgFillColor(s->vg, fillColor);
  nvgFill(s->vg);
}

static void ui_draw_circle_image(const UIState *s, int center_x, int center_y, int radius, const char *image, NVGcolor color, float img_alpha) {
  nvgBeginPath(s->vg);
  nvgCircle(s->vg, center_x, center_y, radius);
  nvgFillColor(s->vg, color);
  nvgFill(s->vg);
  const int img_size = radius * 1.5;
  ui_draw_image(s, {center_x - (img_size / 2), center_y - (img_size / 2), img_size, img_size}, image, img_alpha);
}

static void ui_draw_circle_image(const UIState *s, int center_x, int center_y, int radius, const char *image, bool active) {
  float bg_alpha = active ? 0.3f : 0.1f;
  float img_alpha = active ? 1.0f : 0.15f;
  ui_draw_circle_image(s, center_x, center_y, radius, image, nvgRGBA(0, 0, 0, (255 * bg_alpha)), img_alpha);
}

static void draw_lead(UIState *s, const cereal::RadarState::LeadData::Reader &lead_data, const vertex_data &vd) {
  // Draw lead car indicator
  auto [x, y] = vd;

  float fillAlpha = 0;
  float speedBuff = 10.;
  float leadBuff = 40.;
  float d_rel = lead_data.getDRel();
  float v_rel = lead_data.getVRel();
  if (d_rel < leadBuff) {
    fillAlpha = 255*(1.0-(d_rel/leadBuff));
    if (v_rel < 0) {
      fillAlpha += 255*(-1*(v_rel/speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }

  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  x = std::clamp(x, 0.f, s->fb_w - sz / 2);
  y = std::fmin(s->fb_h - sz * .6, y);
  draw_chevron(s, x, y, sz, nvgRGBA(201, 34, 49, fillAlpha), COLOR_YELLOW);
}

static void ui_draw_line(UIState *s, const line_vertices_data &vd, NVGcolor *color, NVGpaint *paint) {
  if (vd.cnt == 0) return;

  const vertex_data *v = &vd.v[0];
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, v[0].x, v[0].y);
  for (int i = 1; i < vd.cnt; i++) {
    nvgLineTo(s->vg, v[i].x, v[i].y);
  }
  nvgClosePath(s->vg);
  if (color) {
    nvgFillColor(s->vg, *color);
  } else if (paint) {
    nvgFillPaint(s->vg, *paint);
  }
  nvgFill(s->vg);
}

static void draw_vision_frame(UIState *s) {
  glBindVertexArray(s->frame_vao);
  mat4 *out_mat = &s->rear_frame_mat;
  glActiveTexture(GL_TEXTURE0);

  if (s->last_frame) {
    glBindTexture(GL_TEXTURE_2D, s->texture[s->last_frame->idx]->frame_tex);
    if (!Hardware::EON()) {
      // this is handled in ion on QCOM
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, s->last_frame->width, s->last_frame->height,
                   0, GL_RGB, GL_UNSIGNED_BYTE, s->last_frame->addr);
    }
  }

  glUseProgram(s->gl_shader->prog);
  glUniform1i(s->gl_shader->getUniformLocation("uTexture"), 0);
  glUniformMatrix4fv(s->gl_shader->getUniformLocation("uTransform"), 1, GL_TRUE, out_mat->v);

  assert(glGetError() == GL_NO_ERROR);
  glEnableVertexAttribArray(0);
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, (const void *)0);
  glDisableVertexAttribArray(0);
  glBindVertexArray(0);
}

static void ui_draw_vision_lane_lines(UIState *s) {
  const UIScene &scene = s->scene;
  NVGpaint track_bg;
  if (!scene.end_to_end) {
    // paint lanelines
    for (int i = 0; i < std::size(scene.lane_line_vertices); i++) {
      NVGcolor color = nvgRGBAf(1.0, 1.0, 1.0, scene.lane_line_probs[i]);
      ui_draw_line(s, scene.lane_line_vertices[i], &color, nullptr);
    }

    // paint road edges
    for (int i = 0; i < std::size(scene.road_edge_vertices); i++) {
      NVGcolor color = nvgRGBAf(1.0, 0.0, 0.0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0));
      ui_draw_line(s, scene.road_edge_vertices[i], &color, nullptr);
    }
    track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                          COLOR_WHITE, COLOR_WHITE_ALPHA(0));
  } else {
    track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                          COLOR_RED, COLOR_RED_ALPHA(0));
  }
  // paint path
  ui_draw_line(s, scene.track_vertices, nullptr, &track_bg);
}

// Draw all world space objects.
static void ui_draw_world(UIState *s) {
  nvgScissor(s->vg, 0, 0, s->fb_w, s->fb_h);

  // Draw lane edges and vision/mpc tracks
  ui_draw_vision_lane_lines(s);

  // Draw lead indicators if openpilot is handling longitudinal
  if (s->scene.longitudinal_control) {
    auto radar_state = (*s->sm)["radarState"].getRadarState();
    auto lead_one = radar_state.getLeadOne();
    auto lead_two = radar_state.getLeadTwo();
    if (lead_one.getStatus()) {
      draw_lead(s, lead_one, s->scene.lead_vertices[0]);
    }
    if (lead_two.getStatus() && (std::abs(lead_one.getDRel() - lead_two.getDRel()) > 3.0)) {
      draw_lead(s, lead_two, s->scene.lead_vertices[1]);
    }
  }
  nvgResetScissor(s->vg);
}

// 기존 크루즈 셋팅 UI (PSK)
/*static void ui_draw_vision_maxspeed(UIState *s) {
  const int SET_SPEED_NA = 255;

  float maxspeed = (*s->sm)["controlsState"].getControlsState().getVCruise();

  const bool is_cruise_set = maxspeed != 0 && maxspeed != SET_SPEED_NA;
  if (is_cruise_set && !s->scene.is_metric) { maxspeed *= 0.6225; }

  const Rect rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
  ui_fill_rect(s->vg, rect, COLOR_BLACK_ALPHA(100), 30.);
  ui_draw_rect(s->vg, rect, COLOR_WHITE_ALPHA(100), 10, 20.);

  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  ui_draw_text(s, rect.centerX(), 118, "MAX", 26 * 2.5, COLOR_WHITE_ALPHA(is_cruise_set ? 200 : 100), "sans-regular");

  if (is_cruise_set) {
    const std::string maxspeed_str = std::to_string((int)std::nearbyint(maxspeed));
    ui_draw_text(s, rect.centerX(), 212, maxspeed_str.c_str(), 48 * 2.5, COLOR_WHITE, "sans-bold");
  } else {
    ui_draw_text(s, rect.centerX(), 212, "-", 42 * 2.5, COLOR_WHITE_ALPHA(100), "sans-semibold");
  }
}*/


static void ui_draw_vision_maxspeed(UIState *s) {

  // scc smoother
  auto control_state = (*s->sm)["controlsState"].getControlsState();

  // 상단속도 : 적용 속도
  // 하단속도 : 크루즈 속도

  // kph
  float applyMaxSpeed = control_state.getApplyMaxSpeed();
  float cruiseMaxSpeed = control_state.getCruiseMaxSpeed();

  bool is_cruise_set = (cruiseMaxSpeed > 0 && cruiseMaxSpeed < 255);

  const Rect rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
  ui_fill_rect(s->vg, rect, COLOR_BLACK_ALPHA(100), 30.);
  ui_draw_rect(s->vg, rect, COLOR_WHITE_ALPHA(100), 10, 20.);

  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  const int text_x = rect.centerX();

  if(is_cruise_set)
  {
    char str[256];

    if(s->scene.is_metric)
        snprintf(str, sizeof(str), "%d", (int)(applyMaxSpeed + 0.5));
    else
        snprintf(str, sizeof(str), "%d", (int)(applyMaxSpeed*0.621371 + 0.5));

    ui_draw_text(s, text_x, 100, str, 33 * 2.5, COLOR_WHITE, "sans-semibold");

    if(s->scene.is_metric)
        snprintf(str, sizeof(str), "%d", (int)(cruiseMaxSpeed + 0.5));
    else
        snprintf(str, sizeof(str), "%d", (int)(cruiseMaxSpeed*0.621371 + 0.5));

    ui_draw_text(s, text_x, 195, str, 48 * 2.5, COLOR_WHITE, "sans-bold");
  }
  else
  {
    ui_draw_text(s, text_x, 100, "MAX", 25 * 2.5, COLOR_WHITE_ALPHA(100), "sans-semibold");
    ui_draw_text(s, text_x, 195, "N/A", 42 * 2.5, COLOR_WHITE_ALPHA(100), "sans-semibold");
  }
}

static void ui_draw_vision_speed(UIState *s) {
  const float speed = std::max(0.0, (*s->sm)["carState"].getCarState().getVEgo() * (s->scene.is_metric ? 3.6 : 2.2369363));
  const std::string speed_str = std::to_string((int)std::nearbyint(speed));
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  ui_draw_text(s, s->fb_w/2, 210, speed_str.c_str(), 96 * 2.5, COLOR_WHITE, "sans-bold");
  ui_draw_text(s, s->fb_w/2, 290, s->scene.is_metric ? "km/h" : "mph", 36 * 2.5, COLOR_WHITE_ALPHA(200), "sans-regular");
}

static void ui_draw_vision_event(UIState *s) {
  if (s->scene.engageable) {
    // draw steering wheel
    const int radius = 96;
    const int center_x = s->fb_w - radius - bdr_s * 2;
    const int center_y = radius  + (bdr_s * 1.5);
    const QColor &color = bg_colors[s->status];
    NVGcolor nvg_color = nvgRGBA(color.red(), color.green(), color.blue(), color.alpha());
    ui_draw_circle_image(s, center_x, center_y, radius, "wheel", nvg_color, 1.0f);
  }
}

/*static void ui_draw_vision_face(UIState *s) {
  const int radius = 96;
  const int center_x = radius + (bdr_s * 2);
  const int center_y = s->fb_h - footer_h / 2;
  ui_draw_circle_image(s, center_x, center_y, radius, "driver_face", s->scene.dm_active);
}*/

// 1> GAP
static void ui_draw_vision_scc_gap(UIState *s) {
  auto control_state = (*s->sm)["controlsState"].getControlsState();
  int gap = control_state.getDistanceGap();
  if(gap < 0)
    return;

  const int radius = 96;
  const int center_x = radius + (bdr_s * 2) + radius*2 + 60;
  const int center_y = s->fb_h - footer_h / 2;

  //NVGcolor color_bg = nvgRGBA(0, 0, 0, (255 * 0.1f));
  // 배경화면 (흰색 -> 회색)
  NVGcolor color_bg = nvgRGBA(0, 0, 0, 1);

  nvgBeginPath(s->vg);
  nvgCircle(s->vg, center_x, center_y, radius);
  nvgFillColor(s->vg, color_bg);
  nvgFill(s->vg);

  NVGcolor textColor = nvgRGBA(255, 255, 255, 200);
  float textSize = 30.f;

  char str[64];

  if(gap == 0) {
    snprintf(str, sizeof(str), "AUTO");
    textColor = nvgRGBA(120, 255, 120, 200);
  }
  else {
    snprintf(str, sizeof(str), "%d", (int)gap);
    textColor = nvgRGBA(120, 255, 120, 200);
    textSize = 38.f;
  }

  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);

  ui_draw_text(s, center_x, center_y-36, "GAP", 22 * 2.5f, nvgRGBA(255, 255, 255, 200), "sans-bold");
  ui_draw_text(s, center_x, center_y+22, str, textSize * 2.5f, textColor, "sans-bold");

}

// 2> 엑셀 프로파일
static void ui_draw_vision_accel_profile(UIState *s) {
  auto control_state = (*s->sm)["controlsState"].getControlsState();
  int accel = control_state.getAccelProfile();
  if(accel < 0)
    return;

  const int radius = 96;
  const int center_x = radius + (bdr_s * 2) + (radius*2 + 60) * 2;
  const int center_y = s->fb_h - footer_h / 2;

  //NVGcolor color_bg = nvgRGBA(0, 0, 0, (255 * 0.1f));
  NVGcolor color_bg = nvgRGBA(0, 0, 0, 1);

  nvgBeginPath(s->vg);
  nvgCircle(s->vg, center_x, center_y, radius);
  nvgFillColor(s->vg, color_bg);
  nvgFill(s->vg);

  NVGcolor textColor = nvgRGBA(255, 255, 255, 200);
  float textSize = 30.f;

  char str[64];

  if(accel == 0) {
    snprintf(str, sizeof(str), "ECO");
    textColor = nvgRGBA(120, 255, 120, 200);
  } else if(accel == 1) {
    snprintf(str, sizeof(str), "NOR");
    textColor = nvgRGBA(120, 255, 120, 200);
  } else if(accel == 2) {
    snprintf(str, sizeof(str), "SPT");
    textColor = nvgRGBA(120, 255, 120, 200);
  }

  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);

  ui_draw_text(s, center_x, center_y-36, "ACCEL", 22 * 2.5f, nvgRGBA(255, 255, 255, 200), "sans-bold");
  ui_draw_text(s, center_x, center_y+22, str, textSize * 2.5f, textColor, "sans-bold");

}

// 3> ACC
static void ui_draw_vision_acc(UIState *s) {
  auto control_state = (*s->sm)["controlsState"].getControlsState();
  int acc = control_state.getAdaptiveCruise();
  if(acc < 0)
    return;

  const int radius = 96;
  const int center_x = radius + (bdr_s * 2) + (radius*2 + 60) * 3;
  const int center_y = s->fb_h - footer_h / 2;

  NVGcolor color_bg = nvgRGBA(0, 0, 0, 1);

  nvgBeginPath(s->vg);
  nvgCircle(s->vg, center_x, center_y, radius);
  nvgFillColor(s->vg, color_bg);
  nvgFill(s->vg);

  NVGcolor textColor = nvgRGBA(255, 255, 255, 200);
  float textSize = 30.f;

  char str[64];

  if(acc == 0) {
    snprintf(str, sizeof(str), "OFF");
    textColor = nvgRGBA(120, 255, 120, 200);
  } else if(acc == 1) {
    snprintf(str, sizeof(str), "ON");
    textColor = nvgRGBA(120, 255, 120, 200);
  }

  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);

  ui_draw_text(s, center_x, center_y-36, "ACC", 22 * 2.5f, nvgRGBA(255, 255, 255, 200), "sans-bold");
  ui_draw_text(s, center_x, center_y+22, str, textSize * 2.5f, textColor, "sans-bold");

}

// 4> Brake
static void ui_draw_vision_brake(UIState *s) {
  const int radius = 96;
  const int brake_x = radius + (bdr_s * 2) + (radius*2 + 60) * 4;
  const int brake_y = s->fb_h - footer_h / 2;
  ui_draw_circle_image(s, brake_x, brake_y, radius, "brake_img", s->scene.brakeLights);
}

// 5> autohold
static void ui_draw_vision_autohold(UIState *s) {
  auto car_state = (*s->sm)["carState"].getCarState();
  int autohold = car_state.getAutoHold();
  if(autohold < 0)
    return;

  const int radius = 96;
  const int center_x = radius + (bdr_s * 2) + (radius*2 + 60) * 5;
  const int center_y = s->fb_h - footer_h / 2;

  float brake_img_alpha = autohold > 0 ? 1.0f : 0.15f;
  float brake_bg_alpha = autohold > 0 ? 0.3f : 0.1f;
  NVGcolor brake_bg = nvgRGBA(0, 0, 0, (255 * brake_bg_alpha));

  ui_draw_circle_image(s, center_x, center_y, radius,
        autohold > 1 ? "autohold_warning" : "autohold_active",
        brake_bg, brake_img_alpha);
}

static void ui_draw_vision_header(UIState *s) {
  NVGpaint gradient = nvgLinearGradient(s->vg, 0, header_h - (header_h / 2.5), 0, header_h,
                                        nvgRGBAf(0, 0, 0, 0.45), nvgRGBAf(0, 0, 0, 0));
  ui_fill_rect(s->vg, {0, 0, s->fb_w , header_h}, gradient);

  if (s->scene.longitudinal_control) {
    ui_draw_vision_maxspeed(s);
  }
  ui_draw_vision_speed(s);
  ui_draw_vision_event(s);
  // NDA
  ui_draw_extras(s);
}

//BB START: functions added for the display of various items
static int bb_ui_draw_measure(UIState *s,  const char* bb_value, const char* bb_uom, const char* bb_label,
    int bb_x, int bb_y, int bb_uom_dx,
    NVGcolor bb_valueColor, NVGcolor bb_labelColor, NVGcolor bb_uomColor,
    int bb_valueFontSize, int bb_labelFontSize, int bb_uomFontSize )  {
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  int dx = 0;
  if (strlen(bb_uom) > 0) {
    dx = (int)(bb_uomFontSize*2.5/2);
   }
  //print value
  nvgFontFace(s->vg, "sans-semibold");
  nvgFontSize(s->vg, bb_valueFontSize*2.5);
  nvgFillColor(s->vg, bb_valueColor);
  nvgText(s->vg, bb_x-dx/2, bb_y+ (int)(bb_valueFontSize*2.5)+5, bb_value, NULL);
  //print label
  nvgFontFace(s->vg, "sans-regular");
  nvgFontSize(s->vg, bb_labelFontSize*2.5);
  nvgFillColor(s->vg, bb_labelColor);
  nvgText(s->vg, bb_x, bb_y + (int)(bb_valueFontSize*2.5)+5 + (int)(bb_labelFontSize*2.5)+5, bb_label, NULL);
  //print uom
  if (strlen(bb_uom) > 0) {
      nvgSave(s->vg);
    int rx =bb_x + bb_uom_dx + bb_valueFontSize -3;
    int ry = bb_y + (int)(bb_valueFontSize*2.5/2)+25;
    nvgTranslate(s->vg,rx,ry);
    nvgRotate(s->vg, -1.5708); //-90deg in radians
    nvgFontFace(s->vg, "sans-regular");
    nvgFontSize(s->vg, (int)(bb_uomFontSize*2.5));
    nvgFillColor(s->vg, bb_uomColor);
    nvgText(s->vg, 0, 0, bb_uom, NULL);
    nvgRestore(s->vg);
  }
  return (int)((bb_valueFontSize + bb_labelFontSize)*2.5) + 5;
}

static void bb_ui_draw_basic_info(UIState *s)
{
    //const UIScene *scene = &s->scene;
    char str[1024];
    std::string sccLogMessage = "";

    auto controls_state = (*s->sm)["controlsState"].getControlsState();
    //auto car_params = (*s->sm)["carParams"].getCarParams();
    auto live_params = (*s->sm)["liveParameters"].getLiveParameters();

    snprintf(str, sizeof(str), "AO(%.2f/%.2f) SR(%.2f) SRC(%.2f) SAD(%.2f) LAD(%.2f) LAT(%.2f) CURV(%.2f)%s%s",

                        live_params.getAngleOffsetDeg(),
                        live_params.getAngleOffsetAverageDeg(),
                        controls_state.getSteerRatio(),
                        controls_state.getSteerRateCost(),
                        controls_state.getSteerActuatorDelay(),
                        controls_state.getLongitudinalActuatorDelay(),
                        controls_state.getLeadAccelTau(),
                        controls_state.getSccCurvatureFactor(),
                        sccLogMessage.size() > 0 ? ", " : "",
                        sccLogMessage.c_str()
                        );

    int x = bdr_s * 2;
    int y = s->fb_h - 24;

    nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);

    ui_draw_text(s, x, y, str, 20 * 2.5, COLOR_WHITE_ALPHA(200), "sans-semibold");
}

static void bb_ui_draw_measures_left(UIState *s, int bb_x, int bb_y, int bb_w ) {
  int bb_rx = bb_x + (int)(bb_w/2);
  int bb_ry = bb_y;
  int bb_h = 5;
  NVGcolor lab_color = nvgRGBA(255, 255, 255, 200);
  NVGcolor uom_color = nvgRGBA(255, 255, 255, 200);
  int value_fontSize=30;
  int label_fontSize=15;
  int uom_fontSize = 15;
  int bb_uom_dx =  (int)(bb_w /2 - uom_fontSize*2.5) ;

  //add visual radar relative distance
  if (true) {
    char val_str[16];
    char uom_str[6];
    NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
    if (s->scene.lead_status) {
      //show RED if less than 10 meters
      //show orange if less than 30 meters
      if((int)(s->scene.lead_d_rel) < 30) {
        val_color = nvgRGBA(255, 188, 3, 200);
      }
      if((int)(s->scene.lead_d_rel) < 10) {
        val_color = nvgRGBA(255, 0, 0, 200);
      }
      // lead car relative distance is always in meters
      snprintf(val_str, sizeof(val_str), "%d", (int)s->scene.lead_d_rel);
    } else {
       snprintf(val_str, sizeof(val_str), "-");
    }
    snprintf(uom_str, sizeof(uom_str), "m");
    bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "REL DIST",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize );
    bb_ry = bb_y + bb_h;
  }

  //add visual radar relative speed
  if (true) {
    char val_str[16];
    char uom_str[6];
    NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
    if (s->scene.lead_status) {
      //show Orange if negative speed (approaching)
      //show Orange if negative speed faster than 5mph (approaching fast)
      if((int)(s->scene.lead_v_rel) < 0) {
        val_color = nvgRGBA(255, 188, 3, 200);
      }
      if((int)(s->scene.lead_v_rel) < -5) {
        val_color = nvgRGBA(255, 0, 0, 200);
      }
      // lead car relative speed is always in meters
      if (s->scene.is_metric) {
         snprintf(val_str, sizeof(val_str), "%d", (int)(s->scene.lead_v_rel * 3.6 + 0.5));
      } else {
         snprintf(val_str, sizeof(val_str), "%d", (int)(s->scene.lead_v_rel * 2.2374144 + 0.5));
      }
    } else {
       snprintf(val_str, sizeof(val_str), "-");
    }
    if (s->scene.is_metric) {
      snprintf(uom_str, sizeof(uom_str), "km/h");;
    } else {
      snprintf(uom_str, sizeof(uom_str), "mph");
    }
    bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "REL SPEED",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize );
    bb_ry = bb_y + bb_h;
  }

  float batteryTemp = (*s->sm)["deviceState"].getDeviceState().getBatteryTempC();
  bool batteryless =  batteryTemp < -20;

  // add battery level
  if(UI_FEATURE_BATTERY_LEVEL && !batteryless) {
    char val_str[16];
    char uom_str[6];
    //char bat_lvl[4] = "";
    NVGcolor val_color = nvgRGBA(255, 255, 255, 200);

    int batteryPercent = (*s->sm)["deviceState"].getDeviceState().getBatteryPercent();

    snprintf(val_str, sizeof(val_str), "%d%%", batteryPercent);
    snprintf(uom_str, sizeof(uom_str), "");
    bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "BAT LVL",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize );
    bb_ry = bb_y + bb_h;
  }

  float ambientTemp = (*s->sm)["deviceState"].getDeviceState().getAmbientTempC();
  // add ambient temperature
  if (UI_FEATURE_AMBIENT_TEMP) {

    char val_str[16];
    char uom_str[6];
    NVGcolor val_color = nvgRGBA(255, 255, 255, 200);

    if(ambientTemp > 48.f) {
      val_color = nvgRGBA(255, 188, 3, 200);
    }
    if(ambientTemp > 55.f) {
      val_color = nvgRGBA(255, 0, 0, 200);
    }
    snprintf(val_str, sizeof(val_str), "%.1f°", ambientTemp);
    snprintf(uom_str, sizeof(uom_str), "");
    bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "BODY Temp",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize );
    bb_ry = bb_y + bb_h;
  }

  // add CPU temperature
  if(UI_FEATURE_CPU_TEMP) {
    char val_str[16];
    char uom_str[6];
    NVGcolor val_color = nvgRGBA(255, 255, 255, 200);

    float cpuTemp = 0;
    auto cpuList = (*s->sm)["deviceState"].getDeviceState().getCpuTempC();

    if(cpuList.size() > 0)
    {
        for(int i = 0; i < cpuList.size(); i++)
            cpuTemp += cpuList[i];

        cpuTemp /= cpuList.size();
    }

    if(cpuTemp > 80.f) {
      val_color = nvgRGBA(255, 188, 3, 200);
    }
    if(cpuTemp > 92.f) {
      val_color = nvgRGBA(255, 0, 0, 200);
    }

    // temp is alway in C * 10
    snprintf(val_str, sizeof(val_str), "%.1f°", cpuTemp);
    snprintf(uom_str, sizeof(uom_str), "");
    bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "CPU Temp",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize );
    bb_ry = bb_y + bb_h;
  }

  //finally draw the frame
  bb_h += 20;
  nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, bb_x, bb_y, bb_w, bb_h, 20);
    nvgStrokeColor(s->vg, nvgRGBA(255,255,255,80));
    nvgStrokeWidth(s->vg, 6);
    nvgStroke(s->vg);
}

static void bb_ui_draw_UI(UIState *s) {
  //const UIScene *scene = &s->scene;
  const int bb_dml_w = 180;
  const int bb_dml_x = (bdr_s * 2);
  const int bb_dml_y = (bdr_s * 1.5) + 220;

  bb_ui_draw_measures_left(s, bb_dml_x, bb_dml_y, bb_dml_w);

  bb_ui_draw_basic_info(s);
}

static void ui_draw_vision(UIState *s) {
  const UIScene *scene = &s->scene;
  // Draw augmented elements
  if (scene->world_objects_visible) {
    ui_draw_world(s);
  }
  // Set Speed, Current Speed, Status/Events
  ui_draw_vision_header(s);
  if ((*s->sm)["controlsState"].getControlsState().getAlertSize() == cereal::ControlsState::AlertSize::NONE) {
    //ui_draw_vision_face(s);  비활성화
    ui_draw_vision_scc_gap(s);   // 차간거리
    ui_draw_vision_accel_profile(s);  // 엑셀 프로파일
    ui_draw_vision_acc(s);      // ACC 사용 유무
	ui_draw_vision_brake(s);
	// 오토 홀드 추가
	ui_draw_vision_autohold(s);
	bb_ui_draw_UI(s);
  }
}

void ui_draw(UIState *s, int w, int h) {
  const bool draw_vision = s->scene.started && s->vipc_client->connected;

  glViewport(0, 0, s->fb_w, s->fb_h);
  if (draw_vision) {
    draw_vision_frame(s);
  }
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // NVG drawing functions - should be no GL inside NVG frame
  nvgBeginFrame(s->vg, s->fb_w, s->fb_h, 1.0f);
  if (draw_vision) {
    ui_draw_vision(s);
  }
  nvgEndFrame(s->vg);
  glDisable(GL_BLEND);
}

void ui_draw_image(const UIState *s, const Rect &r, const char *name, float alpha) {
  nvgBeginPath(s->vg);
  NVGpaint imgPaint = nvgImagePattern(s->vg, r.x, r.y, r.w, r.h, 0, s->images.at(name), alpha);
  nvgRect(s->vg, r.x, r.y, r.w, r.h);
  nvgFillPaint(s->vg, imgPaint);
  nvgFill(s->vg);
}

void ui_draw_rect(NVGcontext *vg, const Rect &r, NVGcolor color, int width, float radius) {
  nvgBeginPath(vg);
  radius > 0 ? nvgRoundedRect(vg, r.x, r.y, r.w, r.h, radius) : nvgRect(vg, r.x, r.y, r.w, r.h);
  nvgStrokeColor(vg, color);
  nvgStrokeWidth(vg, width);
  nvgStroke(vg);
}

static inline void fill_rect(NVGcontext *vg, const Rect &r, const NVGcolor *color, const NVGpaint *paint, float radius) {
  nvgBeginPath(vg);
  radius > 0 ? nvgRoundedRect(vg, r.x, r.y, r.w, r.h, radius) : nvgRect(vg, r.x, r.y, r.w, r.h);
  if (color) nvgFillColor(vg, *color);
  if (paint) nvgFillPaint(vg, *paint);
  nvgFill(vg);
}
void ui_fill_rect(NVGcontext *vg, const Rect &r, const NVGcolor &color, float radius) {
  fill_rect(vg, r, &color, nullptr, radius);
}
void ui_fill_rect(NVGcontext *vg, const Rect &r, const NVGpaint &paint, float radius) {
  fill_rect(vg, r, nullptr, &paint, radius);
}

static const char frame_vertex_shader[] =
#ifdef NANOVG_GL3_IMPLEMENTATION
  "#version 150 core\n"
#else
  "#version 300 es\n"
#endif
  "in vec4 aPosition;\n"
  "in vec4 aTexCoord;\n"
  "uniform mat4 uTransform;\n"
  "out vec4 vTexCoord;\n"
  "void main() {\n"
  "  gl_Position = uTransform * aPosition;\n"
  "  vTexCoord = aTexCoord;\n"
  "}\n";

static const char frame_fragment_shader[] =
#ifdef NANOVG_GL3_IMPLEMENTATION
  "#version 150 core\n"
#else
  "#version 300 es\n"
#endif
  "precision mediump float;\n"
  "uniform sampler2D uTexture;\n"
  "in vec4 vTexCoord;\n"
  "out vec4 colorOut;\n"
  "void main() {\n"
  "  colorOut = texture(uTexture, vTexCoord.xy);\n"
#ifdef QCOM
  "  vec3 dz = vec3(0.0627f, 0.0627f, 0.0627f);\n"
  "  colorOut.rgb = ((vec3(1.0f, 1.0f, 1.0f) - dz) * colorOut.rgb / vec3(1.0f, 1.0f, 1.0f)) + dz;\n"
#endif
  "}\n";

static const mat4 device_transform = {{
  1.0,  0.0, 0.0, 0.0,
  0.0,  1.0, 0.0, 0.0,
  0.0,  0.0, 1.0, 0.0,
  0.0,  0.0, 0.0, 1.0,
}};

void ui_nvg_init(UIState *s) {
  // init drawing

  // on EON, we enable MSAA
  s->vg = Hardware::EON() ? nvgCreate(0) : nvgCreate(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
  assert(s->vg);

  // init fonts
  std::pair<const char *, const char *> fonts[] = {
      {"sans-regular", "../assets/fonts/opensans_regular.ttf"},
      {"sans-semibold", "../assets/fonts/opensans_semibold.ttf"},
      {"sans-bold", "../assets/fonts/opensans_bold.ttf"},
  };
  for (auto [name, file] : fonts) {
    int font_id = nvgCreateFont(s->vg, name, file);
    assert(font_id >= 0);
  }

  // init images
  std::vector<std::pair<const char *, const char *>> images = {
    {"wheel", "../assets/img_chffr_wheel.png"},
    {"driver_face", "../assets/img_driver_face.png"},
    {"brake_img", "../assets/img_brake_disc.png"},
    {"img_nda", "../assets/img_nda.png"},
	{"img_hda", "../assets/img_hda.png"},
	{"autohold_warning", "../assets/img_autohold_warning.png"},
	{"autohold_active", "../assets/img_autohold_active.png"},
  };
  for (auto [name, file] : images) {
    s->images[name] = nvgCreateImage(s->vg, file, 1);
    assert(s->images[name] != 0);
  }

  // init gl
  s->gl_shader = std::make_unique<GLShader>(frame_vertex_shader, frame_fragment_shader);
  GLint frame_pos_loc = glGetAttribLocation(s->gl_shader->prog, "aPosition");
  GLint frame_texcoord_loc = glGetAttribLocation(s->gl_shader->prog, "aTexCoord");

  glViewport(0, 0, s->fb_w, s->fb_h);

  glDisable(GL_DEPTH_TEST);

  assert(glGetError() == GL_NO_ERROR);

  float x1 = 1.0, x2 = 0.0, y1 = 1.0, y2 = 0.0;
  const uint8_t frame_indicies[] = {0, 1, 2, 0, 2, 3};
  const float frame_coords[4][4] = {
    {-1.0, -1.0, x2, y1}, //bl
    {-1.0,  1.0, x2, y2}, //tl
    { 1.0,  1.0, x1, y2}, //tr
    { 1.0, -1.0, x1, y1}, //br
  };

  glGenVertexArrays(1, &s->frame_vao);
  glBindVertexArray(s->frame_vao);
  glGenBuffers(1, &s->frame_vbo);
  glBindBuffer(GL_ARRAY_BUFFER, s->frame_vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(frame_coords), frame_coords, GL_STATIC_DRAW);
  glEnableVertexAttribArray(frame_pos_loc);
  glVertexAttribPointer(frame_pos_loc, 2, GL_FLOAT, GL_FALSE,
                        sizeof(frame_coords[0]), (const void *)0);
  glEnableVertexAttribArray(frame_texcoord_loc);
  glVertexAttribPointer(frame_texcoord_loc, 2, GL_FLOAT, GL_FALSE,
                        sizeof(frame_coords[0]), (const void *)(sizeof(float) * 2));
  glGenBuffers(1, &s->frame_ibo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, s->frame_ibo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(frame_indicies), frame_indicies, GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);

  ui_resize(s, s->fb_w, s->fb_h);
}

void ui_resize(UIState *s, int width, int height) {
  s->fb_w = width;
  s->fb_h = height;

  auto intrinsic_matrix = s->wide_camera ? ecam_intrinsic_matrix : fcam_intrinsic_matrix;

  float zoom = ZOOM / intrinsic_matrix.v[0];

  if (s->wide_camera) {
    zoom *= 0.5;
  }

  float zx = zoom * 2 * intrinsic_matrix.v[2] / width;
  float zy = zoom * 2 * intrinsic_matrix.v[5] / height;

  const mat4 frame_transform = {{
    zx, 0.0, 0.0, 0.0,
    0.0, zy, 0.0, -y_offset / height * 2,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
  }};

  s->rear_frame_mat = matmul(device_transform, frame_transform);

  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  nvgTranslate(s->vg, width / 2, height / 2 + y_offset);
  // 2) Apply same scaling as video
  nvgScale(s->vg, zoom, zoom);
  // 3) Put (0, 0) in top left corner of video
  nvgTranslate(s->vg, -intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);

  nvgCurrentTransform(s->vg, s->car_space_transform);
  nvgResetTransform(s->vg);
}
