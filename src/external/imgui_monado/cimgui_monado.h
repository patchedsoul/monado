#ifndef CIMGUI_MONADO_INCLUDED
#define CIMGUI_MONADO_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

void igPlotTimings(const char *label,
                   float (*values_getter)(void *data, int idx), void *data,
                   int values_count, int values_offset,
                   const char *overlay_text, float scale_min, float scale_max,
                   ImVec2 frame_size, float reference_timing,
                   bool center_reference_timing, float range, char *unit,
                   bool dynamic_rescale);

#ifdef __cplusplus
}
#endif

#endif // CIMGUI_MONADO_INCLUDED
