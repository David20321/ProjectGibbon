using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace ImGuiNET
{
    public static unsafe partial class ImGuiNative
    {
        [DllImport("cimgui")]
        public static extern float igGetFrameHeight();
        [DllImport("cimgui")]
        public static extern IntPtr igCreateContext(ImFontAtlas* shared_font_atlas);
        [DllImport("cimgui")]
        public static extern void igTextUnformatted(byte* text, byte* text_end);
        [DllImport("cimgui")]
        public static extern void igPopFont();
        [DllImport("cimgui")]
        public static extern byte igCombo(byte* label, int* current_item, byte** items, int items_count, int popup_max_height_in_items);
        [DllImport("cimgui")]
        public static extern byte igComboStr(byte* label, int* current_item, byte* items_separated_by_zeros, int popup_max_height_in_items);
        [DllImport("cimgui")]
        public static extern void igCaptureKeyboardFromApp(byte capture);
        [DllImport("cimgui")]
        public static extern byte igIsWindowFocused(ImGuiFocusedFlags flags);
        [DllImport("cimgui")]
        public static extern void igRender();
        [DllImport("cimgui")]
        public static extern void ImDrawList_ChannelsSetCurrent(ImDrawList* self, int channel_index);
        [DllImport("cimgui")]
        public static extern byte igDragFloat4(byte* label, Vector4* v, float v_speed, float v_min, float v_max, byte* format, float power);
        [DllImport("cimgui")]
        public static extern void ImDrawList_ChannelsSplit(ImDrawList* self, int channels_count);
        [DllImport("cimgui")]
        public static extern byte igIsMousePosValid(Vector2* mouse_pos);
        [DllImport("cimgui", EntryPoint = "igGetCursorScreenPos_nonUDT2")]
        public static extern Vector2 igGetCursorScreenPos();
        [DllImport("cimgui")]
        public static extern byte igDebugCheckVersionAndDataLayout(byte* version_str, uint sz_io, uint sz_style, uint sz_vec2, uint sz_vec4, uint sz_drawvert);
        [DllImport("cimgui")]
        public static extern void igSetScrollHere(float center_y_ratio);
        [DllImport("cimgui")]
        public static extern void igSetScrollY(float scroll_y);
        [DllImport("cimgui")]
        public static extern void igSetColorEditOptions(ImGuiColorEditFlags flags);
        [DllImport("cimgui")]
        public static extern void igSetScrollFromPosY(float pos_y, float center_y_ratio);
        [DllImport("cimgui")]
        public static extern Vector4* igGetStyleColorVec4(ImGuiCol idx);
        [DllImport("cimgui")]
        public static extern byte igIsMouseHoveringRect(Vector2 r_min, Vector2 r_max, byte clip);
        [DllImport("cimgui")]
        public static extern Vector4* ImVec4_ImVec4();
        [DllImport("cimgui")]
        public static extern void ImVec4_ImVec4Float(Vector4* self, float _x, float _y, float _z, float _w);
        [DllImport("cimgui")]
        public static extern void ImColor_SetHSV(ImColor* self, float h, float s, float v, float a);
        [DllImport("cimgui")]
        public static extern byte igDragFloat3(byte* label, Vector3* v, float v_speed, float v_min, float v_max, byte* format, float power);
        [DllImport("cimgui")]
        public static extern void ImDrawList_AddPolyline(ImDrawList* self, Vector2* points, int num_points, uint col, byte closed, float thickness);
        [DllImport("cimgui")]
        public static extern void igValueBool(byte* prefix, byte b);
        [DllImport("cimgui")]
        public static extern void igValueInt(byte* prefix, int v);
        [DllImport("cimgui")]
        public static extern void igValueUint(byte* prefix, uint v);
        [DllImport("cimgui")]
        public static extern void igValueFloat(byte* prefix, float v, byte* float_format);
        [DllImport("cimgui", EntryPoint = "igGetItemRectMax_nonUDT2")]
        public static extern Vector2 igGetItemRectMax();
        [DllImport("cimgui")]
        public static extern byte igIsItemDeactivated();
        [DllImport("cimgui")]
        public static extern void igPushStyleVarFloat(ImGuiStyleVar idx, float val);
        [DllImport("cimgui")]
        public static extern void igPushStyleVarVec2(ImGuiStyleVar idx, Vector2 val);
        [DllImport("cimgui")]
        public static extern byte* igSaveIniSettingsToMemory(uint* out_ini_size);
        [DllImport("cimgui")]
        public static extern byte igDragIntRange2(byte* label, int* v_current_min, int* v_current_max, float v_speed, int v_min, int v_max, byte* format, byte* format_max);
        [DllImport("cimgui")]
        public static extern void igUnindent(float indent_w);
        [DllImport("cimgui")]
        public static extern ImFont* ImFontAtlas_AddFontFromMemoryCompressedBase85TTF(ImFontAtlas* self, byte* compressed_font_data_base85, float size_pixels, ImFontConfigNative* font_cfg, ushort* glyph_ranges);
        [DllImport("cimgui")]
        public static extern void igPopAllowKeyboardFocus();
        [DllImport("cimgui")]
        public static extern void igLoadIniSettingsFromDisk(byte* ini_filename);
        [DllImport("cimgui", EntryPoint = "igGetCursorStartPos_nonUDT2")]
        public static extern Vector2 igGetCursorStartPos();
        [DllImport("cimgui")]
        public static extern void igSetCursorScreenPos(Vector2 screen_pos);
        [DllImport("cimgui")]
        public static extern byte igInputInt4(byte* label, int* v, ImGuiInputTextFlags extra_flags);
        [DllImport("cimgui")]
        public static extern void ImFont_AddRemapChar(ImFont* self, ushort dst, ushort src, byte overwrite_dst);
        [DllImport("cimgui")]
        public static extern void ImFont_AddGlyph(ImFont* self, ushort c, float x0, float y0, float x1, float y1, float u0, float v0, float u1, float v1, float advance_x);
        [DllImport("cimgui")]
        public static extern byte igIsRectVisible(Vector2 size);
        [DllImport("cimgui")]
        public static extern byte igIsRectVisibleVec2(Vector2 rect_min, Vector2 rect_max);
        [DllImport("cimgui")]
        public static extern void ImFont_GrowIndex(ImFont* self, int new_size);
        [DllImport("cimgui")]
        public static extern byte ImFontAtlas_Build(ImFontAtlas* self);
        [DllImport("cimgui")]
        public static extern void igLabelText(byte* label, byte* fmt);
        [DllImport("cimgui")]
        public static extern void ImFont_RenderText(ImFont* self, ImDrawList* draw_list, float size, Vector2 pos, uint col, Vector4 clip_rect, byte* text_begin, byte* text_end, float wrap_width, byte cpu_fine_clip);
        [DllImport("cimgui")]
        public static extern void igLogFinish();
        [DllImport("cimgui")]
        public static extern byte igIsKeyPressed(int user_key_index, byte repeat);
        [DllImport("cimgui")]
        public static extern float igGetColumnOffset(int column_index);
        [DllImport("cimgui")]
        public static extern void ImDrawList_PopClipRect(ImDrawList* self);
        [DllImport("cimgui")]
        public static extern ImFontGlyph* ImFont_FindGlyphNoFallback(ImFont* self, ushort c);
        [DllImport("cimgui")]
        public static extern void igSetNextWindowCollapsed(byte collapsed, ImGuiCond cond);
        [DllImport("cimgui")]
        public static extern IntPtr igGetCurrentContext();
        [DllImport("cimgui")]
        public static extern byte igSmallButton(byte* label);
        [DllImport("cimgui")]
        public static extern byte igOpenPopupOnItemClick(byte* str_id, int mouse_button);
        [DllImport("cimgui")]
        public static extern byte igIsAnyMouseDown();
        [DllImport("cimgui")]
        public static extern byte* ImFont_CalcWordWrapPositionA(ImFont* self, float scale, byte* text, byte* text_end, float wrap_width);
        [DllImport("cimgui", EntryPoint = "ImFont_CalcTextSizeA_nonUDT2")]
        public static extern Vector2 ImFont_CalcTextSizeA(ImFont* self, float size, float max_width, float wrap_width, byte* text_begin, byte* text_end, byte** remaining);
        [DllImport("cimgui")]
        public static extern void GlyphRangesBuilder_SetBit(GlyphRangesBuilder* self, int n);
        [DllImport("cimgui")]
        public static extern byte ImFont_IsLoaded(ImFont* self);
        [DllImport("cimgui")]
        public static extern float ImFont_GetCharAdvance(ImFont* self, ushort c);
        [DllImport("cimgui")]
        public static extern byte igImageButton(IntPtr user_texture_id, Vector2 size, Vector2 uv0, Vector2 uv1, int frame_padding, Vector4 bg_col, Vector4 tint_col);
        [DllImport("cimgui")]
        public static extern void ImFont_SetFallbackChar(ImFont* self, ushort c);
        [DllImport("cimgui")]
        public static extern void igEndFrame();
        [DllImport("cimgui")]
        public static extern byte igSliderFloat2(byte* label, Vector2* v, float v_min, float v_max, byte* format, float power);
        [DllImport("cimgui")]
        public static extern void ImFont_RenderChar(ImFont* self, ImDrawList* draw_list, float size, Vector2 pos, uint col, ushort c);
        [DllImport("cimgui")]
        public static extern byte igRadioButtonBool(byte* label, byte active);
        [DllImport("cimgui")]
        public static extern byte igRadioButtonIntPtr(byte* label, int* v, int v_button);
        [DllImport("cimgui")]
        public static extern void ImDrawList_PushClipRect(ImDrawList* self, Vector2 clip_rect_min, Vector2 clip_rect_max, byte intersect_with_current_clip_rect);
        [DllImport("cimgui")]
        public static extern ImFontGlyph* ImFont_FindGlyph(ImFont* self, ushort c);
        [DllImport("cimgui")]
        public static extern byte igIsItemDeactivatedAfterEdit();
        [DllImport("cimgui")]
        public static extern ImDrawList* igGetWindowDrawList();
        [DllImport("cimgui")]
        public static extern ImFont* ImFontAtlas_AddFont(ImFontAtlas* self, ImFontConfigNative* font_cfg);
        [DllImport("cimgui")]
        public static extern void ImDrawList_PathBezierCurveTo(ImDrawList* self, Vector2 p1, Vector2 p2, Vector2 p3, int num_segments);
        [DllImport("cimgui")]
        public static extern void ImGuiPayload_Clear(ImGuiPayload* self);
        [DllImport("cimgui")]
        public static extern void igNewLine();
        [DllImport("cimgui")]
        public static extern byte igIsItemFocused();
        [DllImport("cimgui")]
        public static extern void igLoadIniSettingsFromMemory(byte* ini_data, uint ini_size);
        [DllImport("cimgui")]
        public static extern byte igSliderInt2(byte* label, int* v, int v_min, int v_max, byte* format);
        [DllImport("cimgui")]
        public static extern void igSetWindowSizeVec2(Vector2 size, ImGuiCond cond);
        [DllImport("cimgui")]
        public static extern void igSetWindowSizeStr(byte* name, Vector2 size, ImGuiCond cond);
        [DllImport("cimgui")]
        public static extern byte igInputFloat(byte* label, float* v, float step, float step_fast, byte* format, ImGuiInputTextFlags extra_flags);
        [DllImport("cimgui")]
        public static extern ImFont* ImFont_ImFont();
        [DllImport("cimgui")]
        public static extern void ImGuiStorage_SetFloat(ImGuiStorage* self, uint key, float val);
        [DllImport("cimgui")]
        public static extern void igColorConvertRGBtoHSV(float r, float g, float b, float* out_h, float* out_s, float* out_v);
        [DllImport("cimgui")]
        public static extern byte igBeginMenuBar();
        [DllImport("cimgui")]
        public static extern byte igIsPopupOpen(byte* str_id);
        [DllImport("cimgui")]
        public static extern byte igIsItemVisible();
        [DllImport("cimgui")]
        public static extern void ImFontAtlas_CalcCustomRectUV(ImFontAtlas* self, CustomRect* rect, Vector2* out_uv_min, Vector2* out_uv_max);
        [DllImport("cimgui")]
        public static extern CustomRect* ImFontAtlas_GetCustomRectByIndex(ImFontAtlas* self, int index);
        [DllImport("cimgui")]
        public static extern void GlyphRangesBuilder_AddText(GlyphRangesBuilder* self, byte* text, byte* text_end);
        [DllImport("cimgui")]
        public static extern void ImDrawList_UpdateTextureID(ImDrawList* self);
        [DllImport("cimgui")]
        public static extern void igSetNextWindowSize(Vector2 size, ImGuiCond cond);
        [DllImport("cimgui")]
        public static extern int ImFontAtlas_AddCustomRectRegular(ImFontAtlas* self, uint id, int width, int height);
        [DllImport("cimgui")]
        public static extern void igSetWindowCollapsedBool(byte collapsed, ImGuiCond cond);
        [DllImport("cimgui")]
        public static extern void igSetWindowCollapsedStr(byte* name, byte collapsed, ImGuiCond cond);
        [DllImport("cimgui", EntryPoint = "igGetMouseDragDelta_nonUDT2")]
        public static extern Vector2 igGetMouseDragDelta(int button, float lock_threshold);
        [DllImport("cimgui")]
        public static extern ImGuiPayload* igAcceptDragDropPayload(byte* type, ImGuiDragDropFlags flags);
        [DllImport("cimgui")]
        public static extern byte igBeginDragDropSource(ImGuiDragDropFlags flags);
        [DllImport("cimgui")]
        public static extern byte CustomRect_IsPacked(CustomRect* self);
        [DllImport("cimgui")]
        public static extern void igPlotLines(byte* label, float* values, int values_count, int values_offset, byte* overlay_text, float scale_min, float scale_max, Vector2 graph_size, int stride);
        [DllImport("cimgui")]
        public static extern byte ImFontAtlas_IsBuilt(ImFontAtlas* self);
        [DllImport("cimgui")]
        public static extern Vector2* ImVec2_ImVec2();
        [DllImport("cimgui")]
        public static extern void ImVec2_ImVec2Float(Vector2* self, float _x, float _y);
        [DllImport("cimgui")]
        public static extern ImGuiPayload* ImGuiPayload_ImGuiPayload();
        [DllImport("cimgui")]
        public static extern void ImDrawList_Clear(ImDrawList* self);
        [DllImport("cimgui")]
        public static extern void GlyphRangesBuilder_AddRanges(GlyphRangesBuilder* self, ushort* ranges);
        [DllImport("cimgui")]
        public static extern int igGetFrameCount();
        [DllImport("cimgui")]
        public static extern byte* ImFont_GetDebugName(ImFont* self);
        [DllImport("cimgui")]
        public static extern void igListBoxFooter();
        [DllImport("cimgui")]
        public static extern void igPopClipRect();
        [DllImport("cimgui")]
        public static extern void ImDrawList_AddBezierCurve(ImDrawList* self, Vector2 pos0, Vector2 cp0, Vector2 cp1, Vector2 pos1, uint col, float thickness, int num_segments);
        [DllImport("cimgui")]
        public static extern GlyphRangesBuilder* GlyphRangesBuilder_GlyphRangesBuilder();
        [DllImport("cimgui", EntryPoint = "igGetWindowSize_nonUDT2")]
        public static extern Vector2 igGetWindowSize();
        [DllImport("cimgui")]
        public static extern ushort* ImFontAtlas_GetGlyphRangesThai(ImFontAtlas* self);
        [DllImport("cimgui")]
        public static extern byte igCheckboxFlags(byte* label, uint* flags, uint flags_value);
        [DllImport("cimgui")]
        public static extern ushort* ImFontAtlas_GetGlyphRangesCyrillic(ImFontAtlas* self);
        [DllImport("cimgui")]
        public static extern byte igIsWindowHovered(ImGuiHoveredFlags flags);
        [DllImport("cimgui")]
        public static extern ushort* ImFontAtlas_GetGlyphRangesChineseSimplifiedCommon(ImFontAtlas* self);
        [DllImport("cimgui")]
        public static extern void igPlotHistogramFloatPtr(byte* label, float* values, int values_count, int values_offset, byte* overlay_text, float scale_min, float scale_max, Vector2 graph_size, int stride);
        [DllImport("cimgui")]
        public static extern byte igBeginPopupContextVoid(byte* str_id, int mouse_button);
        [DllImport("cimgui")]
        public static extern ushort* ImFontAtlas_GetGlyphRangesChineseFull(ImFontAtlas* self);
        [DllImport("cimgui")]
        public static extern void igShowStyleEditor(ImGuiStyle* @ref);
        [DllImport("cimgui")]
        public static extern byte igCheckbox(byte* label, byte* v);
        [DllImport("cimgui", EntryPoint = "igGetWindowPos_nonUDT2")]
        public static extern Vector2 igGetWindowPos();
        [DllImport("cimgui")]
        public static extern ImGuiInputTextCallbackDataNative* ImGuiInputTextCallbackData_ImGuiInputTextCallbackData();
        [DllImport("cimgui")]
        public static extern void igSetNextWindowContentSize(Vector2 size);
        [DllImport("cimgui")]
        public static extern void igTextColored(Vector4 col, byte* fmt);
        [DllImport("cimgui")]
        public static extern void igLogToFile(int max_depth, byte* filename);
        [DllImport("cimgui")]
        public static extern byte igButton(byte* label, Vector2 size);
        [DllImport("cimgui")]
        public static extern byte igIsItemEdited();
        [DllImport("cimgui")]
        public static extern void ImDrawList_PushTextureID(ImDrawList* self, IntPtr texture_id);
        [DllImport("cimgui")]
        public static extern void igTreeAdvanceToLabelPos();
        [DllImport("cimgui")]
        public static extern void ImGuiInputTextCallbackData_DeleteChars(ImGuiInputTextCallbackDataNative* self, int pos, int bytes_count);
        [DllImport("cimgui")]
        public static extern byte igDragInt2(byte* label, int* v, float v_speed, int v_min, int v_max, byte* format);
        [DllImport("cimgui")]
        public static extern ushort* ImFontAtlas_GetGlyphRangesDefault(ImFontAtlas* self);
        [DllImport("cimgui")]
        public static extern byte igIsAnyItemActive();
        [DllImport("cimgui")]
        public static extern void ImFontAtlas_SetTexID(ImFontAtlas* self, IntPtr id);
        [DllImport("cimgui")]
        public static extern byte igMenuItemBool(byte* label, byte* shortcut, byte selected, byte enabled);
        [DllImport("cimgui")]
        public static extern byte igMenuItemBoolPtr(byte* label, byte* shortcut, byte* p_selected, byte enabled);
        [DllImport("cimgui")]
        public static extern byte igSliderFloat4(byte* label, Vector4* v, float v_min, float v_max, byte* format, float power);
        [DllImport("cimgui")]
        public static extern float igGetCursorPosX();
        [DllImport("cimgui")]
        public static extern void ImFontAtlas_ClearTexData(ImFontAtlas* self);
        [DllImport("cimgui")]
        public static extern void ImFontAtlas_ClearFonts(ImFontAtlas* self);
        [DllImport("cimgui")]
        public static extern int igGetColumnsCount();
        [DllImport("cimgui")]
        public static extern void igPopButtonRepeat();
        [DllImport("cimgui")]
        public static extern byte igDragScalarN(byte* label, ImGuiDataType data_type, void* v, int components, float v_speed, void* v_min, void* v_max, byte* format, float power);
        [DllImport("cimgui")]
        public static extern byte ImGuiPayload_IsPreview(ImGuiPayload* self);
        [DllImport("cimgui")]
        public static extern void igSpacing();
        [DllImport("cimgui")]
        public static extern void ImFontAtlas_Clear(ImFontAtlas* self);
        [DllImport("cimgui")]
        public static extern byte igIsAnyItemFocused();
        [DllImport("cimgui")]
        public static extern void ImDrawList_AddRectFilled(ImDrawList* self, Vector2 a, Vector2 b, uint col, float rounding, int rounding_corners_flags);
        [DllImport("cimgui")]
        public static extern ImFont* ImFontAtlas_AddFontFromMemoryCompressedTTF(ImFontAtlas* self, void* compressed_font_data, int compressed_font_size, float size_pixels, ImFontConfigNative* font_cfg, ushort* glyph_ranges);
        [DllImport("cimgui")]
        public static extern void igMemFree(void* ptr);
        [DllImport("cimgui", EntryPoint = "igGetFontTexUvWhitePixel_nonUDT2")]
        public static extern Vector2 igGetFontTexUvWhitePixel();
        [DllImport("cimgui")]
        public static extern void ImDrawList_AddDrawCmd(ImDrawList* self);
        [DllImport("cimgui")]
        public static extern byte igIsItemClicked(int mouse_button);
        [DllImport("cimgui")]
        public static extern ImFont* ImFontAtlas_AddFontFromMemoryTTF(ImFontAtlas* self, void* font_data, int font_size, float size_pixels, ImFontConfigNative* font_cfg, ushort* glyph_ranges);
        [DllImport("cimgui")]
        public static extern ImFont* ImFontAtlas_AddFontFromFileTTF(ImFontAtlas* self, byte* filename, float size_pixels, ImFontConfigNative* font_cfg, ushort* glyph_ranges);
        [DllImport("cimgui")]
        public static extern void igProgressBar(float fraction, Vector2 size_arg, byte* overlay);
        [DllImport("cimgui")]
        public static extern ImFont* ImFontAtlas_AddFontDefault(ImFontAtlas* self, ImFontConfigNative* font_cfg);
        [DllImport("cimgui")]
        public static extern void igSetNextWindowBgAlpha(float alpha);
        [DllImport("cimgui")]
        public static extern byte igBeginPopup(byte* str_id, ImGuiWindowFlags flags);
        [DllImport("cimgui")]
        public static extern void ImFont_BuildLookupTable(ImFont* self);
        [DllImport("cimgui")]
        public static extern float igGetScrollX();
        [DllImport("cimgui")]
        public static extern int igGetKeyIndex(ImGuiKey imgui_key);
        [DllImport("cimgui")]
        public static extern ImDrawList* igGetOverlayDrawList();
        [DllImport("cimgui")]
        public static extern uint igGetIDStr(byte* str_id);
        [DllImport("cimgui")]
        public static extern uint igGetIDStrStr(byte* str_id_begin, byte* str_id_end);
        [DllImport("cimgui")]
        public static extern uint igGetIDPtr(void* ptr_id);
        [DllImport("cimgui")]
        public static extern ushort* ImFontAtlas_GetGlyphRangesJapanese(ImFontAtlas* self);
        [DllImport("cimgui")]
        public static extern byte igListBoxHeaderVec2(byte* label, Vector2 size);
        [DllImport("cimgui")]
        public static extern byte igListBoxHeaderInt(byte* label, int items_count, int height_in_items);
        [DllImport("cimgui")]
        public static extern ImFontConfigNative* ImFontConfig_ImFontConfig();
        [DllImport("cimgui")]
        public static extern void ImFontConfig_destroy(ImFontConfigNative* self);
        [DllImport("cimgui")]
        public static extern byte igIsMouseReleased(int button);
        [DllImport("cimgui")]
        public static extern void ImDrawData_ScaleClipRects(ImDrawData* self, Vector2 sc);
        [DllImport("cimgui", EntryPoint = "igGetItemRectMin_nonUDT2")]
        public static extern Vector2 igGetItemRectMin();
        [DllImport("cimgui")]
        public static extern void ImDrawData_DeIndexAllBuffers(ImDrawData* self);
        [DllImport("cimgui")]
        public static extern void igLogText(byte* fmt);
        [DllImport("cimgui")]
        public static extern void ImDrawData_Clear(ImDrawData* self);
        [DllImport("cimgui")]
        public static extern void* ImGuiStorage_GetVoidPtr(ImGuiStorage* self, uint key);
        [DllImport("cimgui")]
        public static extern void igTextWrapped(byte* fmt);
        [DllImport("cimgui")]
        public static extern void ImDrawList_UpdateClipRect(ImDrawList* self);
        [DllImport("cimgui")]
        public static extern void ImDrawList_PrimVtx(ImDrawList* self, Vector2 pos, Vector2 uv, uint col);
        [DllImport("cimgui")]
        public static extern void igEndGroup();
        [DllImport("cimgui")]
        public static extern ImFont* igGetFont();
        [DllImport("cimgui")]
        public static extern void igTreePushStr(byte* str_id);
        [DllImport("cimgui")]
        public static extern void igTreePushPtr(void* ptr_id);
        [DllImport("cimgui")]
        public static extern void igTextDisabled(byte* fmt);
        [DllImport("cimgui")]
        public static extern void ImDrawList_PrimRect(ImDrawList* self, Vector2 a, Vector2 b, uint col);
        [DllImport("cimgui")]
        public static extern void ImDrawList_AddQuad(ImDrawList* self, Vector2 a, Vector2 b, Vector2 c, Vector2 d, uint col, float thickness);
        [DllImport("cimgui")]
        public static extern void ImDrawList_ClearFreeMemory(ImDrawList* self);
        [DllImport("cimgui")]
        public static extern void igSetNextTreeNodeOpen(byte is_open, ImGuiCond cond);
        [DllImport("cimgui")]
        public static extern void igLogToTTY(int max_depth);
        [DllImport("cimgui")]
        public static extern void GlyphRangesBuilder_BuildRanges(GlyphRangesBuilder* self, ImVector* out_ranges);
        [DllImport("cimgui")]
        public static extern ImDrawList* ImDrawList_CloneOutput(ImDrawList* self);
        [DllImport("cimgui")]
        public static extern ImGuiIO* igGetIO();
        [DllImport("cimgui")]
        public static extern byte igDragInt4(byte* label, int* v, float v_speed, int v_min, int v_max, byte* format);
        [DllImport("cimgui")]
        public static extern void igNextColumn();
        [DllImport("cimgui")]
        public static extern void ImDrawList_AddRect(ImDrawList* self, Vector2 a, Vector2 b, uint col, float rounding, int rounding_corners_flags, float thickness);
        [DllImport("cimgui")]
        public static extern void igSetCursorPos(Vector2 local_pos);
        [DllImport("cimgui")]
        public static extern byte igBeginPopupModal(byte* name, byte* p_open, ImGuiWindowFlags flags);
        [DllImport("cimgui")]
        public static extern byte igSliderInt4(byte* label, int* v, int v_min, int v_max, byte* format);
        [DllImport("cimgui")]
        public static extern void ImDrawList_AddCallback(ImDrawList* self, IntPtr callback, void* callback_data);
        [DllImport("cimgui")]
        public static extern void igShowMetricsWindow(byte* p_open);
        [DllImport("cimgui")]
        public static extern float igGetScrollMaxY();
        [DllImport("cimgui")]
        public static extern void igBeginTooltip();
        [DllImport("cimgui")]
        public static extern void igSetScrollX(float scroll_x);
        [DllImport("cimgui")]
        public static extern ImDrawData* igGetDrawData();
        [DllImport("cimgui")]
        public static extern float igGetTextLineHeight();
        [DllImport("cimgui")]
        public static extern void igSeparator();
        [DllImport("cimgui")]
        public static extern byte igBeginChild(byte* str_id, Vector2 size, byte border, ImGuiWindowFlags flags);
        [DllImport("cimgui")]
        public static extern byte igBeginChildID(uint id, Vector2 size, byte border, ImGuiWindowFlags flags);
        [DllImport("cimgui")]
        public static extern void ImDrawList_PathRect(ImDrawList* self, Vector2 rect_min, Vector2 rect_max, float rounding, int rounding_corners_flags);
        [DllImport("cimgui")]
        public static extern byte igIsMouseClicked(int button, byte repeat);
        [DllImport("cimgui")]
        public static extern float igCalcItemWidth();
        [DllImport("cimgui")]
        public static extern void ImDrawList_PathArcToFast(ImDrawList* self, Vector2 centre, float radius, int a_min_of_12, int a_max_of_12);
        [DllImport("cimgui")]
        public static extern void igEndChildFrame();
        [DllImport("cimgui")]
        public static extern void igIndent(float indent_w);
        [DllImport("cimgui")]
        public static extern byte igSetDragDropPayload(byte* type, void* data, uint size, ImGuiCond cond);
        [DllImport("cimgui")]
        public static extern byte GlyphRangesBuilder_GetBit(GlyphRangesBuilder* self, int n);
        [DllImport("cimgui")]
        public static extern void igShowDemoWindow(byte* p_open);
        [DllImport("cimgui")]
        public static extern void ImDrawList_PathStroke(ImDrawList* self, uint col, byte closed, float thickness);
        [DllImport("cimgui")]
        public static extern void ImDrawList_PathFillConvex(ImDrawList* self, uint col);
        [DllImport("cimgui")]
        public static extern void ImDrawList_PathLineToMergeDuplicate(ImDrawList* self, Vector2 pos);
        [DllImport("cimgui")]
        public static extern void igEndMenu();
        [DllImport("cimgui")]
        public static extern byte igColorButton(byte* desc_id, Vector4 col, ImGuiColorEditFlags flags, Vector2 size);
        [DllImport("cimgui")]
        public static extern void ImFontAtlas_GetTexDataAsAlpha8(ImFontAtlas* self, byte** out_pixels, int* out_width, int* out_height, int* out_bytes_per_pixel);
        [DllImport("cimgui")]
        public static extern byte igIsKeyReleased(int user_key_index);
        [DllImport("cimgui")]
        public static extern void igSetClipboardText(byte* text);
        [DllImport("cimgui")]
        public static extern void ImDrawList_PathArcTo(ImDrawList* self, Vector2 centre, float radius, float a_min, float a_max, int num_segments);
        [DllImport("cimgui")]
        public static extern void ImDrawList_AddConvexPolyFilled(ImDrawList* self, Vector2* points, int num_points, uint col);
        [DllImport("cimgui")]
        public static extern byte igIsWindowCollapsed();
        [DllImport("cimgui")]
        public static extern void igShowFontSelector(byte* label);
        [DllImport("cimgui")]
        public static extern void ImDrawList_AddImageQuad(ImDrawList* self, IntPtr user_texture_id, Vector2 a, Vector2 b, Vector2 c, Vector2 d, Vector2 uv_a, Vector2 uv_b, Vector2 uv_c, Vector2 uv_d, uint col);
        [DllImport("cimgui")]
        public static extern void igSetNextWindowFocus();
        [DllImport("cimgui")]
        public static extern void igSameLine(float pos_x, float spacing_w);
        [DllImport("cimgui")]
        public static extern byte igBegin(byte* name, byte* p_open, ImGuiWindowFlags flags);
        [DllImport("cimgui")]
        public static extern byte igColorEdit3(byte* label, Vector3* col, ImGuiColorEditFlags flags);
        [DllImport("cimgui")]
        public static extern void ImDrawList_AddImage(ImDrawList* self, IntPtr user_texture_id, Vector2 a, Vector2 b, Vector2 uv_a, Vector2 uv_b, uint col);
        [DllImport("cimgui")]
        public static extern void ImGuiIO_AddInputCharactersUTF8(ImGuiIO* self, byte* utf8_chars);
        [DllImport("cimgui")]
        public static extern void ImDrawList_AddText(ImDrawList* self, Vector2 pos, uint col, byte* text_begin, byte* text_end);
        [DllImport("cimgui")]
        public static extern void ImDrawList_AddTextFontPtr(ImDrawList* self, ImFont* font, float font_size, Vector2 pos, uint col, byte* text_begin, byte* text_end, float wrap_width, Vector4* cpu_fine_clip_rect);
        [DllImport("cimgui")]
        public static extern void ImDrawList_AddCircleFilled(ImDrawList* self, Vector2 centre, float radius, uint col, int num_segments);
        [DllImport("cimgui")]
        public static extern byte igInputFloat2(byte* label, Vector2* v, byte* format, ImGuiInputTextFlags extra_flags);
        [DllImport("cimgui")]
        public static extern void igPushButtonRepeat(byte repeat);
        [DllImport("cimgui")]
        public static extern void igPopItemWidth();
        [DllImport("cimgui")]
        public static extern void ImDrawList_AddCircle(ImDrawList* self, Vector2 centre, float radius, uint col, int num_segments, float thickness);
        [DllImport("cimgui")]
        public static extern void ImDrawList_AddTriangleFilled(ImDrawList* self, Vector2 a, Vector2 b, Vector2 c, uint col);
        [DllImport("cimgui")]
        public static extern void ImDrawList_AddTriangle(ImDrawList* self, Vector2 a, Vector2 b, Vector2 c, uint col, float thickness);
        [DllImport("cimgui")]
        public static extern void ImDrawList_AddQuadFilled(ImDrawList* self, Vector2 a, Vector2 b, Vector2 c, Vector2 d, uint col);
        [DllImport("cimgui")]
        public static extern float igGetFontSize();
        [DllImport("cimgui")]
        public static extern byte igInputDouble(byte* label, double* v, double step, double step_fast, byte* format, ImGuiInputTextFlags extra_flags);
        [DllImport("cimgui")]
        public static extern void ImDrawList_PrimReserve(ImDrawList* self, int idx_count, int vtx_count);
        [DllImport("cimgui")]
        public static extern void ImDrawList_AddRectFilledMultiColor(ImDrawList* self, Vector2 a, Vector2 b, uint col_upr_left, uint col_upr_right, uint col_bot_right, uint col_bot_left);
        [DllImport("cimgui")]
        public static extern void igEndPopup();
        [DllImport("cimgui")]
        public static extern void ImFontAtlas_ClearInputData(ImFontAtlas* self);
        [DllImport("cimgui")]
        public static extern void ImDrawList_AddLine(ImDrawList* self, Vector2 a, Vector2 b, uint col, float thickness);
        [DllImport("cimgui")]
        public static extern byte igInputTextMultiline(byte* label, byte* buf, uint buf_size, Vector2 size, ImGuiInputTextFlags flags, ImGuiInputTextCallback callback, void* user_data);
        [DllImport("cimgui")]
        public static extern byte igSelectable(byte* label, byte selected, ImGuiSelectableFlags flags, Vector2 size);
        [DllImport("cimgui")]
        public static extern byte igSelectableBoolPtr(byte* label, byte* p_selected, ImGuiSelectableFlags flags, Vector2 size);
        [DllImport("cimgui")]
        public static extern byte igListBoxStr_arr(byte* label, int* current_item, byte** items, int items_count, int height_in_items);
        [DllImport("cimgui", EntryPoint = "igGetCursorPos_nonUDT2")]
        public static extern Vector2 igGetCursorPos();
        [DllImport("cimgui", EntryPoint = "ImDrawList_GetClipRectMin_nonUDT2")]
        public static extern Vector2 ImDrawList_GetClipRectMin(ImDrawList* self);
        [DllImport("cimgui")]
        public static extern void ImDrawList_PopTextureID(ImDrawList* self);
        [DllImport("cimgui")]
        public static extern byte igInputFloat4(byte* label, Vector4* v, byte* format, ImGuiInputTextFlags extra_flags);
        [DllImport("cimgui")]
        public static extern void igSetCursorPosY(float y);
        [DllImport("cimgui")]
        public static extern byte* igGetVersion();
        [DllImport("cimgui")]
        public static extern void igEndCombo();
        [DllImport("cimgui")]
        public static extern void igPushIDStr(byte* str_id);
        [DllImport("cimgui")]
        public static extern void igPushIDRange(byte* str_id_begin, byte* str_id_end);
        [DllImport("cimgui")]
        public static extern void igPushIDPtr(void* ptr_id);
        [DllImport("cimgui")]
        public static extern void igPushIDInt(int int_id);
        [DllImport("cimgui")]
        public static extern ImDrawList* ImDrawList_ImDrawList(IntPtr shared_data);
        [DllImport("cimgui")]
        public static extern ImDrawCmd* ImDrawCmd_ImDrawCmd();
        [DllImport("cimgui")]
        public static extern void ImGuiListClipper_End(ImGuiListClipperNative* self);
        [DllImport("cimgui")]
        public static extern void igAlignTextToFramePadding();
        [DllImport("cimgui")]
        public static extern void igPopStyleColor(int count);
        [DllImport("cimgui")]
        public static extern void ImGuiListClipper_Begin(ImGuiListClipperNative* self, int items_count, float items_height);
        [DllImport("cimgui")]
        public static extern void igText(byte* fmt);
        [DllImport("cimgui")]
        public static extern byte ImGuiListClipper_Step(ImGuiListClipperNative* self);
        [DllImport("cimgui")]
        public static extern float igGetTextLineHeightWithSpacing();
        [DllImport("cimgui")]
        public static extern void ImGuiListClipper_destroy(ImGuiListClipperNative* self);
        [DllImport("cimgui")]
        public static extern float* ImGuiStorage_GetFloatRef(ImGuiStorage* self, uint key, float default_val);
        [DllImport("cimgui")]
        public static extern void igEndTooltip();
        [DllImport("cimgui")]
        public static extern ImGuiListClipperNative* ImGuiListClipper_ImGuiListClipper(int items_count, float items_height);
        [DllImport("cimgui")]
        public static extern byte igDragInt(byte* label, int* v, float v_speed, int v_min, int v_max, byte* format);
        [DllImport("cimgui")]
        public static extern byte igSliderFloat(byte* label, float* v, float v_min, float v_max, byte* format, float power);
        [DllImport("cimgui")]
        public static extern uint igColorConvertFloat4ToU32(Vector4 @in);
        [DllImport("cimgui")]
        public static extern void ImGuiIO_ClearInputCharacters(ImGuiIO* self);
        [DllImport("cimgui")]
        public static extern void igPushClipRect(Vector2 clip_rect_min, Vector2 clip_rect_max, byte intersect_with_current_clip_rect);
        [DllImport("cimgui")]
        public static extern void igSetColumnWidth(int column_index, float width);
        [DllImport("cimgui")]
        public static extern byte ImGuiPayload_IsDataType(ImGuiPayload* self, byte* type);
        [DllImport("cimgui")]
        public static extern byte igBeginMainMenuBar();
        [DllImport("cimgui")]
        public static extern CustomRect* CustomRect_CustomRect();
        [DllImport("cimgui")]
        public static extern byte ImGuiInputTextCallbackData_HasSelection(ImGuiInputTextCallbackDataNative* self);
        [DllImport("cimgui")]
        public static extern void ImGuiInputTextCallbackData_InsertChars(ImGuiInputTextCallbackDataNative* self, int pos, byte* text, byte* text_end);
        [DllImport("cimgui")]
        public static extern byte ImFontAtlas_GetMouseCursorTexData(ImFontAtlas* self, ImGuiMouseCursor cursor, Vector2* out_offset, Vector2* out_size, Vector2* out_uv_border, Vector2* out_uv_fill);
        [DllImport("cimgui")]
        public static extern byte igVSliderScalar(byte* label, Vector2 size, ImGuiDataType data_type, void* v, void* v_min, void* v_max, byte* format, float power);
        [DllImport("cimgui")]
        public static extern void ImGuiStorage_SetAllInt(ImGuiStorage* self, int val);
        [DllImport("cimgui")]
        public static extern void** ImGuiStorage_GetVoidPtrRef(ImGuiStorage* self, uint key, void* default_val);
        [DllImport("cimgui")]
        public static extern void igStyleColorsLight(ImGuiStyle* dst);
        [DllImport("cimgui")]
        public static extern byte igSliderFloat3(byte* label, Vector3* v, float v_min, float v_max, byte* format, float power);
        [DllImport("cimgui")]
        public static extern byte igDragFloat(byte* label, float* v, float v_speed, float v_min, float v_max, byte* format, float power);
        [DllImport("cimgui")]
        public static extern byte* ImGuiStorage_GetBoolRef(ImGuiStorage* self, uint key, byte default_val);
        [DllImport("cimgui")]
        public static extern float igGetWindowHeight();
        [DllImport("cimgui", EntryPoint = "igGetMousePosOnOpeningCurrentPopup_nonUDT2")]
        public static extern Vector2 igGetMousePosOnOpeningCurrentPopup();
        [DllImport("cimgui")]
        public static extern int* ImGuiStorage_GetIntRef(ImGuiStorage* self, uint key, int default_val);
        [DllImport("cimgui")]
        public static extern void igCalcListClipping(int items_count, float items_height, int* out_items_display_start, int* out_items_display_end);
        [DllImport("cimgui")]
        public static extern void ImGuiStorage_SetVoidPtr(ImGuiStorage* self, uint key, void* val);
        [DllImport("cimgui")]
        public static extern void igEndDragDropSource();
        [DllImport("cimgui")]
        public static extern void ImGuiStorage_BuildSortByKey(ImGuiStorage* self);
        [DllImport("cimgui")]
        public static extern float ImGuiStorage_GetFloat(ImGuiStorage* self, uint key, float default_val);
        [DllImport("cimgui")]
        public static extern void ImGuiStorage_SetBool(ImGuiStorage* self, uint key, byte val);
        [DllImport("cimgui")]
        public static extern byte ImGuiStorage_GetBool(ImGuiStorage* self, uint key, byte default_val);
        [DllImport("cimgui")]
        public static extern float igGetFrameHeightWithSpacing();
        [DllImport("cimgui")]
        public static extern void ImGuiStorage_SetInt(ImGuiStorage* self, uint key, int val);
        [DllImport("cimgui")]
        public static extern void igCloseCurrentPopup();
        [DllImport("cimgui")]
        public static extern void igBeginGroup();
        [DllImport("cimgui")]
        public static extern void ImGuiStorage_Clear(ImGuiStorage* self);
        [DllImport("cimgui")]
        public static extern void Pair_PairInt(Pair* self, uint _key, int _val_i);
        [DllImport("cimgui")]
        public static extern void Pair_PairFloat(Pair* self, uint _key, float _val_f);
        [DllImport("cimgui")]
        public static extern void Pair_PairPtr(Pair* self, uint _key, void* _val_p);
        [DllImport("cimgui")]
        public static extern byte igSliderScalar(byte* label, ImGuiDataType data_type, void* v, void* v_min, void* v_max, byte* format, float power);
        [DllImport("cimgui")]
        public static extern byte igBeginCombo(byte* label, byte* preview_value, ImGuiComboFlags flags);
        [DllImport("cimgui")]
        public static extern byte igBeginMenu(byte* label, byte enabled);
        [DllImport("cimgui")]
        public static extern byte igIsItemHovered(ImGuiHoveredFlags flags);
        [DllImport("cimgui")]
        public static extern void ImDrawList_PrimWriteVtx(ImDrawList* self, Vector2 pos, Vector2 uv, uint col);
        [DllImport("cimgui")]
        public static extern void igBullet();
        [DllImport("cimgui")]
        public static extern byte igInputText(byte* label, byte* buf, uint buf_size, ImGuiInputTextFlags flags, ImGuiInputTextCallback callback, void* user_data);
        [DllImport("cimgui")]
        public static extern byte igInputInt3(byte* label, int* v, ImGuiInputTextFlags extra_flags);
        [DllImport("cimgui")]
        public static extern ImGuiIO* ImGuiIO_ImGuiIO();
        [DllImport("cimgui")]
        public static extern void igStyleColorsDark(ImGuiStyle* dst);
        [DllImport("cimgui")]
        public static extern byte igInputInt(byte* label, int* v, int step, int step_fast, ImGuiInputTextFlags extra_flags);
        [DllImport("cimgui")]
        public static extern void igSetWindowFontScale(float scale);
        [DllImport("cimgui")]
        public static extern byte igSliderInt(byte* label, int* v, int v_min, int v_max, byte* format);
        [DllImport("cimgui")]
        public static extern void igSetNextWindowPos(Vector2 pos, ImGuiCond cond, Vector2 pivot);
        [DllImport("cimgui")]
        public static extern byte igDragInt3(byte* label, int* v, float v_speed, int v_min, int v_max, byte* format);
        [DllImport("cimgui")]
        public static extern void igOpenPopup(byte* str_id);
        [DllImport("cimgui", EntryPoint = "ImDrawList_GetClipRectMax_nonUDT2")]
        public static extern Vector2 ImDrawList_GetClipRectMax(ImDrawList* self);
        [DllImport("cimgui", EntryPoint = "igCalcTextSize_nonUDT2")]
        public static extern Vector2 igCalcTextSize(byte* text, byte* text_end, byte hide_text_after_double_hash, float wrap_width);
        [DllImport("cimgui")]
        public static extern IntPtr igGetDrawListSharedData();
        [DllImport("cimgui")]
        public static extern void igColumns(int count, byte* id, byte border);
        [DllImport("cimgui")]
        public static extern byte igIsItemActive();
        [DllImport("cimgui")]
        public static extern ImGuiOnceUponAFrame* ImGuiOnceUponAFrame_ImGuiOnceUponAFrame();
        [DllImport("cimgui")]
        public static extern byte igBeginDragDropTarget();
        [DllImport("cimgui")]
        public static extern byte ImGuiPayload_IsDelivery(ImGuiPayload* self);
        [DllImport("cimgui")]
        public static extern void ImGuiIO_AddInputCharacter(ImGuiIO* self, ushort c);
        [DllImport("cimgui")]
        public static extern void ImDrawList_AddImageRounded(ImDrawList* self, IntPtr user_texture_id, Vector2 a, Vector2 b, Vector2 uv_a, Vector2 uv_b, uint col, float rounding, int rounding_corners);
        [DllImport("cimgui")]
        public static extern ImGuiStyle* ImGuiStyle_ImGuiStyle();
        [DllImport("cimgui")]
        public static extern byte igColorPicker3(byte* label, Vector3* col, ImGuiColorEditFlags flags);
        [DllImport("cimgui", EntryPoint = "igGetContentRegionMax_nonUDT2")]
        public static extern Vector2 igGetContentRegionMax();
        [DllImport("cimgui")]
        public static extern byte igBeginChildFrame(uint id, Vector2 size, ImGuiWindowFlags flags);
        [DllImport("cimgui")]
        public static extern void igSaveIniSettingsToDisk(byte* ini_filename);
        [DllImport("cimgui")]
        public static extern void ImFont_ClearOutputData(ImFont* self);
        [DllImport("cimgui")]
        public static extern byte* igGetClipboardText();
        [DllImport("cimgui")]
        public static extern void ImDrawList_PrimQuadUV(ImDrawList* self, Vector2 a, Vector2 b, Vector2 c, Vector2 d, Vector2 uv_a, Vector2 uv_b, Vector2 uv_c, Vector2 uv_d, uint col);
        [DllImport("cimgui")]
        public static extern void igEndDragDropTarget();
        [DllImport("cimgui")]
        public static extern ushort* ImFontAtlas_GetGlyphRangesKorean(ImFontAtlas* self);
        [DllImport("cimgui")]
        public static extern int igGetKeyPressedAmount(int key_index, float repeat_delay, float rate);
        [DllImport("cimgui")]
        public static extern void ImFontAtlas_GetTexDataAsRGBA32(ImFontAtlas* self, byte** out_pixels, int* out_width, int* out_height, int* out_bytes_per_pixel);
        [DllImport("cimgui")]
        public static extern void igNewFrame();
        [DllImport("cimgui")]
        public static extern void igResetMouseDragDelta(int button);
        [DllImport("cimgui")]
        public static extern float igGetTreeNodeToLabelSpacing();
        [DllImport("cimgui", EntryPoint = "igGetMousePos_nonUDT2")]
        public static extern Vector2 igGetMousePos();
        [DllImport("cimgui")]
        public static extern void GlyphRangesBuilder_AddChar(GlyphRangesBuilder* self, ushort c);
        [DllImport("cimgui")]
        public static extern void igPopID();
        [DllImport("cimgui")]
        public static extern byte igIsMouseDoubleClicked(int button);
        [DllImport("cimgui")]
        public static extern void igStyleColorsClassic(ImGuiStyle* dst);
        [DllImport("cimgui")]
        public static extern void ImDrawList_PathClear(ImDrawList* self);
        [DllImport("cimgui")]
        public static extern void igSetWindowFocus();
        [DllImport("cimgui")]
        public static extern void igSetWindowFocusStr(byte* name);
        [DllImport("cimgui")]
        public static extern void igColorConvertHSVtoRGB(float h, float s, float v, float* out_r, float* out_g, float* out_b);
        [DllImport("cimgui")]
        public static extern ImColor* ImColor_ImColor();
        [DllImport("cimgui")]
        public static extern void ImColor_ImColorInt(ImColor* self, int r, int g, int b, int a);
        [DllImport("cimgui")]
        public static extern void ImColor_ImColorU32(ImColor* self, uint rgba);
        [DllImport("cimgui")]
        public static extern void ImColor_ImColorFloat(ImColor* self, float r, float g, float b, float a);
        [DllImport("cimgui")]
        public static extern void ImColor_ImColorVec4(ImColor* self, Vector4 col);
        [DllImport("cimgui")]
        public static extern byte igVSliderFloat(byte* label, Vector2 size, float* v, float v_min, float v_max, byte* format, float power);
        [DllImport("cimgui", EntryPoint = "igColorConvertU32ToFloat4_nonUDT2")]
        public static extern Vector4 igColorConvertU32ToFloat4(uint @in);
        [DllImport("cimgui")]
        public static extern void igPopTextWrapPos();
        [DllImport("cimgui")]
        public static extern ImGuiStorage* igGetStateStorage();
        [DllImport("cimgui")]
        public static extern float igGetColumnWidth(int column_index);
        [DllImport("cimgui")]
        public static extern void igEndMenuBar();
        [DllImport("cimgui")]
        public static extern void igSetStateStorage(ImGuiStorage* storage);
        [DllImport("cimgui")]
        public static extern byte* igGetStyleColorName(ImGuiCol idx);
        [DllImport("cimgui")]
        public static extern byte igIsMouseDragging(int button, float lock_threshold);
        [DllImport("cimgui")]
        public static extern void ImDrawList_PrimWriteIdx(ImDrawList* self, ushort idx);
        [DllImport("cimgui")]
        public static extern void ImGuiStyle_ScaleAllSizes(ImGuiStyle* self, float scale_factor);
        [DllImport("cimgui")]
        public static extern void igPushStyleColorU32(ImGuiCol idx, uint col);
        [DllImport("cimgui")]
        public static extern void igPushStyleColor(ImGuiCol idx, Vector4 col);
        [DllImport("cimgui")]
        public static extern void* igMemAlloc(uint size);
        [DllImport("cimgui")]
        public static extern void igSetCurrentContext(IntPtr ctx);
        [DllImport("cimgui")]
        public static extern void igPushItemWidth(float item_width);
        [DllImport("cimgui")]
        public static extern byte igIsWindowAppearing();
        [DllImport("cimgui")]
        public static extern ImGuiStyle* igGetStyle();
        [DllImport("cimgui")]
        public static extern void igSetItemAllowOverlap();
        [DllImport("cimgui")]
        public static extern void igEndChild();
        [DllImport("cimgui")]
        public static extern byte igCollapsingHeader(byte* label, ImGuiTreeNodeFlags flags);
        [DllImport("cimgui")]
        public static extern byte igCollapsingHeaderBoolPtr(byte* label, byte* p_open, ImGuiTreeNodeFlags flags);
        [DllImport("cimgui")]
        public static extern byte igDragFloatRange2(byte* label, float* v_current_min, float* v_current_max, float v_speed, float v_min, float v_max, byte* format, byte* format_max, float power);
        [DllImport("cimgui")]
        public static extern void igSetMouseCursor(ImGuiMouseCursor type);
        [DllImport("cimgui", EntryPoint = "igGetWindowContentRegionMax_nonUDT2")]
        public static extern Vector2 igGetWindowContentRegionMax();
        [DllImport("cimgui")]
        public static extern byte igInputScalar(byte* label, ImGuiDataType data_type, void* v, void* step, void* step_fast, byte* format, ImGuiInputTextFlags extra_flags);
        [DllImport("cimgui")]
        public static extern void ImDrawList_PushClipRectFullScreen(ImDrawList* self);
        [DllImport("cimgui")]
        public static extern uint igGetColorU32(ImGuiCol idx, float alpha_mul);
        [DllImport("cimgui")]
        public static extern uint igGetColorU32Vec4(Vector4 col);
        [DllImport("cimgui")]
        public static extern uint igGetColorU32U32(uint col);
        [DllImport("cimgui")]
        public static extern double igGetTime();
        [DllImport("cimgui")]
        public static extern void ImDrawList_ChannelsMerge(ImDrawList* self);
        [DllImport("cimgui")]
        public static extern int igGetColumnIndex();
        [DllImport("cimgui")]
        public static extern byte igBeginPopupContextItem(byte* str_id, int mouse_button);
        [DllImport("cimgui")]
        public static extern void igSetCursorPosX(float x);
        [DllImport("cimgui", EntryPoint = "igGetItemRectSize_nonUDT2")]
        public static extern Vector2 igGetItemRectSize();
        [DllImport("cimgui")]
        public static extern byte igArrowButton(byte* str_id, ImGuiDir dir);
        [DllImport("cimgui")]
        public static extern ImGuiMouseCursor igGetMouseCursor();
        [DllImport("cimgui")]
        public static extern void igPushAllowKeyboardFocus(byte allow_keyboard_focus);
        [DllImport("cimgui")]
        public static extern float igGetScrollY();
        [DllImport("cimgui")]
        public static extern void igSetColumnOffset(int column_index, float offset_x);
        [DllImport("cimgui")]
        public static extern void igSetWindowPosVec2(Vector2 pos, ImGuiCond cond);
        [DllImport("cimgui")]
        public static extern void igSetWindowPosStr(byte* name, Vector2 pos, ImGuiCond cond);
        [DllImport("cimgui")]
        public static extern void igSetKeyboardFocusHere(int offset);
        [DllImport("cimgui")]
        public static extern float igGetCursorPosY();
        [DllImport("cimgui")]
        public static extern int ImFontAtlas_AddCustomRectFontGlyph(ImFontAtlas* self, ImFont* font, ushort id, int width, int height, float advance_x, Vector2 offset);
        [DllImport("cimgui")]
        public static extern void igEndMainMenuBar();
        [DllImport("cimgui")]
        public static extern float igGetContentRegionAvailWidth();
        [DllImport("cimgui")]
        public static extern byte igIsKeyDown(int user_key_index);
        [DllImport("cimgui")]
        public static extern byte igIsMouseDown(int button);
        [DllImport("cimgui", EntryPoint = "igGetWindowContentRegionMin_nonUDT2")]
        public static extern Vector2 igGetWindowContentRegionMin();
        [DllImport("cimgui")]
        public static extern void igLogButtons();
        [DllImport("cimgui")]
        public static extern float igGetWindowContentRegionWidth();
        [DllImport("cimgui")]
        public static extern byte igSliderAngle(byte* label, float* v_rad, float v_degrees_min, float v_degrees_max);
        [DllImport("cimgui")]
        public static extern byte igTreeNodeExStr(byte* label, ImGuiTreeNodeFlags flags);
        [DllImport("cimgui")]
        public static extern byte igTreeNodeExStrStr(byte* str_id, ImGuiTreeNodeFlags flags, byte* fmt);
        [DllImport("cimgui")]
        public static extern byte igTreeNodeExPtr(void* ptr_id, ImGuiTreeNodeFlags flags, byte* fmt);
        [DllImport("cimgui")]
        public static extern float igGetWindowWidth();
        [DllImport("cimgui")]
        public static extern void igPushTextWrapPos(float wrap_pos_x);
        [DllImport("cimgui")]
        public static extern int ImGuiStorage_GetInt(ImGuiStorage* self, uint key, int default_val);
        [DllImport("cimgui")]
        public static extern byte igSliderInt3(byte* label, int* v, int v_min, int v_max, byte* format);
        [DllImport("cimgui")]
        public static extern void igShowUserGuide();
        [DllImport("cimgui")]
        public static extern byte igSliderScalarN(byte* label, ImGuiDataType data_type, void* v, int components, void* v_min, void* v_max, byte* format, float power);
        [DllImport("cimgui", EntryPoint = "ImColor_HSV_nonUDT2")]
        public static extern ImColor ImColor_HSV(ImColor* self, float h, float s, float v, float a);
        [DllImport("cimgui")]
        public static extern void ImDrawList_PathLineTo(ImDrawList* self, Vector2 pos);
        [DllImport("cimgui")]
        public static extern void igImage(IntPtr user_texture_id, Vector2 size, Vector2 uv0, Vector2 uv1, Vector4 tint_col, Vector4 border_col);
        [DllImport("cimgui")]
        public static extern void igSetNextWindowSizeConstraints(Vector2 size_min, Vector2 size_max, ImGuiSizeCallback custom_callback, void* custom_callback_data);
        [DllImport("cimgui")]
        public static extern void igDummy(Vector2 size);
        [DllImport("cimgui")]
        public static extern byte igVSliderInt(byte* label, Vector2 size, int* v, int v_min, int v_max, byte* format);
        [DllImport("cimgui")]
        public static extern void igBulletText(byte* fmt);
        [DllImport("cimgui")]
        public static extern byte igColorEdit4(byte* label, Vector4* col, ImGuiColorEditFlags flags);
        [DllImport("cimgui")]
        public static extern byte igColorPicker4(byte* label, Vector4* col, ImGuiColorEditFlags flags, float* ref_col);
        [DllImport("cimgui")]
        public static extern void ImDrawList_PrimRectUV(ImDrawList* self, Vector2 a, Vector2 b, Vector2 uv_a, Vector2 uv_b, uint col);
        [DllImport("cimgui")]
        public static extern byte igInvisibleButton(byte* str_id, Vector2 size);
        [DllImport("cimgui")]
        public static extern void igLogToClipboard(int max_depth);
        [DllImport("cimgui")]
        public static extern byte igBeginPopupContextWindow(byte* str_id, int mouse_button, byte also_over_items);
        [DllImport("cimgui")]
        public static extern ImFontAtlas* ImFontAtlas_ImFontAtlas();
        [DllImport("cimgui")]
        public static extern byte igDragScalar(byte* label, ImGuiDataType data_type, void* v, float v_speed, void* v_min, void* v_max, byte* format, float power);
        [DllImport("cimgui")]
        public static extern void igSetItemDefaultFocus();
        [DllImport("cimgui")]
        public static extern void igCaptureMouseFromApp(byte capture);
        [DllImport("cimgui")]
        public static extern byte igIsAnyItemHovered();
        [DllImport("cimgui")]
        public static extern void igPushFont(ImFont* font);
        [DllImport("cimgui")]
        public static extern byte igInputInt2(byte* label, int* v, ImGuiInputTextFlags extra_flags);
        [DllImport("cimgui")]
        public static extern void igTreePop();
        [DllImport("cimgui")]
        public static extern void igEnd();
        [DllImport("cimgui")]
        public static extern ImDrawData* ImDrawData_ImDrawData();
        [DllImport("cimgui")]
        public static extern void igDestroyContext(IntPtr ctx);
        [DllImport("cimgui")]
        public static extern void igPopStyleVar(int count);
        [DllImport("cimgui")]
        public static extern byte igShowStyleSelector(byte* label);
        [DllImport("cimgui")]
        public static extern byte igInputScalarN(byte* label, ImGuiDataType data_type, void* v, int components, void* step, void* step_fast, byte* format, ImGuiInputTextFlags extra_flags);
        [DllImport("cimgui")]
        public static extern byte igTreeNodeStr(byte* label);
        [DllImport("cimgui")]
        public static extern byte igTreeNodeStrStr(byte* str_id, byte* fmt);
        [DllImport("cimgui")]
        public static extern byte igTreeNodePtr(void* ptr_id, byte* fmt);
        [DllImport("cimgui")]
        public static extern float igGetScrollMaxX();
        [DllImport("cimgui")]
        public static extern void igSetTooltip(byte* fmt);
        [DllImport("cimgui", EntryPoint = "igGetContentRegionAvail_nonUDT2")]
        public static extern Vector2 igGetContentRegionAvail();
        [DllImport("cimgui")]
        public static extern byte igInputFloat3(byte* label, Vector3* v, byte* format, ImGuiInputTextFlags extra_flags);
        [DllImport("cimgui")]
        public static extern byte igDragFloat2(byte* label, Vector2* v, float v_speed, float v_min, float v_max, byte* format, float power);
        [DllImport("cimgui")]
        public static extern byte igBuildFontAtlas(ImFontAtlas* atlas, ImRasterizerFlags extra_flags);
    }
}
