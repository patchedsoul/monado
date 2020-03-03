#include "../imgui/imgui.h"

#ifndef IMGUI_DEFINE_MATH_OPERATORS
#define IMGUI_DEFINE_MATH_OPERATORS
#endif
#include "../imgui/imgui_internal.h"

#include "cimgui_monado.h"

using namespace ImGui;

static void _draw_line(ImGuiWindow *window, int values_count, float scale_min,
                       float scale_max, float val, const ImRect inner_bb,
                       ImVec2 frame_size, ImU32 color) {
  const float inv_scale =
      (scale_min == scale_max) ? 0.0f : (1.0f / (scale_max - scale_min));
  ImVec2 tp0 = ImVec2(
      0.0f, 1.0f - ImSaturate((val - scale_min) *
                              inv_scale)); // Point in the normalized space of
  ImVec2 pos0 = ImLerp(inner_bb.Min, inner_bb.Max, tp0);
  const ImVec2 tp1 =
      ImVec2(1.0, 1.0f - ImSaturate((val - scale_min) * inv_scale));
  ImVec2 pos1 = ImLerp(inner_bb.Min, inner_bb.Max, tp1);
  window->DrawList->AddLine(pos0, pos1, color);

  char text[100];
  snprintf(text, 60, "%6.2f ms", val);
  ImVec2 text_size = ImGui::CalcTextSize(text);
  ImVec2 text_pos = {pos1.x - text_size.x, pos1.y};
  ImGui::PushStyleColor(ImGuiCol_Text, color);
  ImGui::RenderText(text_pos, text);
  ImGui::PopStyleColor(1);
}
static void _draw_grid(ImGuiWindow *window, int values_count, float scale_min,
                       float scale_max, float target_frametime,
                       const ImRect inner_bb, ImVec2 frame_size) {

  ImVec4 target_color = ImVec4(1.0f, 1.0f, 0.0f, .75f);
  _draw_line(window, values_count, scale_min, scale_max, target_frametime,
             inner_bb, frame_size, GetColorU32(target_color));

  ImVec4 passive_color = ImVec4(0.35f, 0.35f, 0.35f, 1.00f);
  for (int i = scale_min + 5; i < scale_max; i += 5) {
    _draw_line(window, values_count, scale_min, scale_max, i, inner_bb,
               frame_size, GetColorU32(passive_color));
  }
}

static void _PlotFrametime(const char *label,
                           float (*values_getter)(void *data, int idx),
                           void *data, int values_count, int values_offset,
                           const char *overlay_text, ImVec2 frame_size,
                           float target_frametime) {
  ImGuiWindow *window = GetCurrentWindow();
  if (window->SkipItems)
    return;

  ImGuiContext &g = *GImGui;
  const ImGuiStyle &style = g.Style;
  const ImGuiID id = window->GetID(label);

  const ImVec2 label_size = CalcTextSize(label, NULL, true);
  if (frame_size.x == 0.0f)
    frame_size.x = CalcItemWidth();
  if (frame_size.y == 0.0f)
    frame_size.y = label_size.y + (style.FramePadding.y * 2);

  const ImRect frame_bb(window->DC.CursorPos,
                        window->DC.CursorPos + frame_size);
  const ImRect inner_bb(frame_bb.Min + style.FramePadding,
                        frame_bb.Max - style.FramePadding);
  const ImRect total_bb(frame_bb.Min,
                        frame_bb.Max +
                            ImVec2(label_size.x > 0.0f
                                       ? style.ItemInnerSpacing.x + label_size.x
                                       : 0.0f,
                                   0));
  ItemSize(total_bb, style.FramePadding.y);
  if (!ItemAdd(total_bb, 0, &frame_bb))
    return;
  const bool hovered = ItemHoverable(frame_bb, id);

  float v_min = FLT_MAX;
  float v_max = -FLT_MAX;
  for (int i = 0; i < values_count; i++) {
    const float v = values_getter(data, i);
    if (v != v) // Ignore NaN values
      continue;
    v_min = ImMin(v_min, v);
    v_max = ImMax(v_max, v);
  }

  /* default scaling is target frametime rounded to next 10 in each direction */
  float scale_min = target_frametime - 11;
  float scale_max = target_frametime + 11;

  /*
  if (v_min < scale_min)
    scale_min = v_min;
  if (v_max > scale_max)
    scale_max = v_max;
  scale_min = ((int)(scale_min / 10)) * 10;
  scale_max = ((int)(scale_max / 10 + 1)) * 10;
  */

  RenderFrame(frame_bb.Min, frame_bb.Max, GetColorU32(ImGuiCol_FrameBg), true,
              style.FrameRounding);

  _draw_grid(window, values_count, scale_min, scale_max, target_frametime,
             inner_bb, frame_size);

  ImGuiPlotType plot_type = ImGuiPlotType_Lines;
  const int values_count_min = (plot_type == ImGuiPlotType_Lines) ? 2 : 1;
  if (values_count >= values_count_min) {
    int res_w = ImMin((int)frame_size.x, values_count) +
                ((plot_type == ImGuiPlotType_Lines) ? -1 : 0);
    int item_count =
        values_count + ((plot_type == ImGuiPlotType_Lines) ? -1 : 0);

    // Tooltip on hover
    int v_hovered = -1;
    if (hovered && inner_bb.Contains(g.IO.MousePos)) {
      const float t = ImClamp((g.IO.MousePos.x - inner_bb.Min.x) /
                                  (inner_bb.Max.x - inner_bb.Min.x),
                              0.0f, 0.9999f);
      const int v_idx = (int)(t * item_count);
      IM_ASSERT(v_idx >= 0 && v_idx < values_count);

      const float v0 =
          values_getter(data, (v_idx + values_offset) % values_count);
      const float v1 =
          values_getter(data, (v_idx + 1 + values_offset) % values_count);
      if (plot_type == ImGuiPlotType_Lines)
        SetTooltip("%d: %8.4g\n%d: %8.4g", v_idx, v0, v_idx + 1, v1);
      else if (plot_type == ImGuiPlotType_Histogram)
        SetTooltip("%d: %8.4g", v_idx, v0);
      v_hovered = v_idx;
    }

    const float t_step = 1.0f / (float)res_w;
    const float inv_scale =
        (scale_min == scale_max) ? 0.0f : (1.0f / (scale_max - scale_min));

    float v0 = values_getter(data, (0 + values_offset) % values_count);
    float t0 = 0.0f;
    ImVec2 tp0 = ImVec2(
        t0, 1.0f - ImSaturate((v0 - scale_min) *
                              inv_scale)); // Point in the normalized space of
                                           // our target rectangle
    float histogram_zero_line_t =
        (scale_min * scale_max < 0.0f)
            ? (-scale_min * inv_scale)
            : (scale_min < 0.0f ? 0.0f
                                : 1.0f); // Where does the zero line stands

    const ImU32 col_base = GetColorU32((plot_type == ImGuiPlotType_Lines)
                                           ? ImGuiCol_PlotLines
                                           : ImGuiCol_PlotHistogram);
    const ImU32 col_hovered = GetColorU32((plot_type == ImGuiPlotType_Lines)
                                              ? ImGuiCol_PlotLinesHovered
                                              : ImGuiCol_PlotHistogramHovered);

    for (int n = 0; n < res_w; n++) {
      const float t1 = t0 + t_step;
      const int v1_idx = (int)(t0 * item_count + 0.5f);
      IM_ASSERT(v1_idx >= 0 && v1_idx < values_count);
      const float v1 =
          values_getter(data, (v1_idx + values_offset + 1) % values_count);
      const ImVec2 tp1 =
          ImVec2(t1, 1.0f - ImSaturate((v1 - scale_min) * inv_scale));

      // NB: Draw calls are merged together by the DrawList system. Still, we
      // should render our batch are lower level to save a bit of CPU.
      ImVec2 pos0 = ImLerp(inner_bb.Min, inner_bb.Max, tp0);
      ImVec2 pos1 = ImLerp(inner_bb.Min, inner_bb.Max,
                           (plot_type == ImGuiPlotType_Lines)
                               ? tp1
                               : ImVec2(tp1.x, histogram_zero_line_t));

      if (plot_type == ImGuiPlotType_Lines) {
        window->DrawList->AddLine(pos0, pos1,
                                  v_hovered == v1_idx ? col_hovered : col_base);
      } else if (plot_type == ImGuiPlotType_Histogram) {
        if (pos1.x >= pos0.x + 2.0f)
          pos1.x -= 1.0f;
        window->DrawList->AddRectFilled(
            pos0, pos1, v_hovered == v1_idx ? col_hovered : col_base);
      }

      t0 = t1;
      tp0 = tp1;
    }
  }

  // Text overlay
  if (overlay_text)
    RenderTextClipped(
        ImVec2(frame_bb.Min.x, frame_bb.Min.y + style.FramePadding.y),
        frame_bb.Max, overlay_text, NULL, NULL, ImVec2(0.5f, 0.0f));

  if (label_size.x > 0.0f)
    RenderText(
        ImVec2(frame_bb.Max.x + style.ItemInnerSpacing.x, inner_bb.Min.y),
        label);

  const float v = values_getter(data, (values_offset));
  ImGui::LabelText("Frame Time", "%6.2fms [%6.2f, %6.2f]", v, v_min, v_max);
}

extern "C" {
void PlotFrametime(const char *label,
                   float (*values_getter)(void *data, int idx), void *data,
                   int values_count, int values_offset,
                   const char *overlay_text, float scale_min, float scale_max,
                   ImVec2 frame_size, float target_frametime) {
  _PlotFrametime(label, values_getter, data, values_count, values_offset,
                 overlay_text, frame_size, target_frametime);
}
}
