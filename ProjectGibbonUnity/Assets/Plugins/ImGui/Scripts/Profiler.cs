using System;
using System.Collections.Generic;
using UnityEngine;
using ImGuiNET;

namespace Wolfire {
    public class Profiler {
        static Profiler instance = new Profiler();
        public static Context special_context;
        static int context_id_index = 0;
        static Context default_context = new Context();
        static Context curr_context = default_context;

        public class Event {
            public string name;
            public double start;
            public double end;
            public int depth;
        }

        public static bool window_visible = false;
        public static bool window_interactive = false;
        
        public const int max_depth = 20;
        public const int max_events = 1024*1024;

        public class Context {
            public System.Diagnostics.Stopwatch stopwatch = System.Diagnostics.Stopwatch.StartNew();
            public int curr_depth = 0;
            public Event[] events = new Event[max_events];
            public Event[] event_stack = new Event[max_depth];
            public int num_events = 0;
            public int id;

            public Context() {            
                for (int i = 0; i < max_events; ++i) {
                    events[i] = new Event();
                }
                id = context_id_index++;
            }

            public int Begin(string val) {
                Debug.Assert(num_events < max_events - 1);
                var evnt = events[num_events];
                evnt.start = stopwatch.Elapsed.TotalSeconds;
                evnt.name = val;
                evnt.depth = curr_depth;
                event_stack[curr_depth] = evnt;
                ++num_events;
                ++curr_depth;
                Debug.Assert(curr_depth < max_depth);
                return num_events - 1;
            }

            public void Begin(int val) {
                var evnt = events[val];
                var total = evnt.end - evnt.start;
                evnt.start = stopwatch.Elapsed.TotalSeconds - total;
            }

            public void End() {
                --curr_depth;
                Debug.Assert(curr_depth >= 0);
                event_stack[curr_depth].end = stopwatch.Elapsed.TotalSeconds;
            }

            public void End(int val) {
                var evnt = events[val];
                evnt.end = stopwatch.Elapsed.TotalSeconds;
            }

            public int Create(string val) {
                int id = Begin(val);
                End();
                return id;
            }

        }
        
        // For use with functions that may have multiple exit points, like this:
        // using (new ScopedEvent("Name of region")) {
        public class ScopedEvent : IDisposable {
            public ScopedEvent(string val){
                Begin(val);
            }
            public ScopedEvent(int val){
                Begin(val);
            }
            public void Dispose() {
                End();
            }
        }

        public Profiler() {
            instance = this;
        }

        public static void StartFrame() {
            curr_context.num_events = 0;
            curr_context.curr_depth = 0;
            Begin("Frame");
        }

        public static Event GetEvent(int id) {
            return curr_context.events[id];
        }


        // Return index to this event in case we want to accumulate
        public static int Begin(string val) {
            return curr_context.Begin(val);
        }

        public static int Create(string val) {
            return curr_context.Create(val);
        }

        public static void Begin(int val) {
            curr_context.Begin(val);
        }

        public static void End() {
            curr_context.End();
        }

        public static void End(int val) {
            curr_context.End(val);
        }


        public static void Display() {
            instance.DisplayInternal();
        }

        double last_display_start;
        double last_display_end;

        void InsertDisplayTimeInternal() {
            int id = Begin("Profiler.Display()");
            End();
            Event display_evnt = curr_context.events[id];
            display_evnt.start = last_display_start;
            display_evnt.end = last_display_end;
        }

        public static void InsertDisplayTime() {
            instance.InsertDisplayTimeInternal();
        }
        
        // From https://answers.unity.com/questions/43422/how-to-implement-show-in-explorer.html
        public void ShowExplorer(string itemPath) {
            itemPath = itemPath.Replace(@"/", @"\");   // explorer doesn't like front slashes
            System.Diagnostics.Process.Start("explorer.exe", "/select,"+itemPath);
        }

        void DisplayProfilerContext(Context context){
            int show_depth = 0;
            ImGui.Indent(ImGui.GetTreeNodeToLabelSpacing());

            for (int i = 0; i < context.num_events; ++i) {
                Event evnt = context.events[i];
                if (evnt.depth <= show_depth) {
                    while (show_depth > evnt.depth) {
                        ImGui.TreePop();
                        --show_depth;
                    }
                    bool is_tree = false;
                    if (i != context.num_events - 1) {
                        Event next_evnt = context.events[i + 1];
                        if (next_evnt.depth > evnt.depth) {
                            is_tree = true;
                        }
                    }
                    if (is_tree) {
                        ImGui.Unindent(ImGui.GetTreeNodeToLabelSpacing());
                        ImGui.SetNextTreeNodeOpen(true);
                        if (ImGui.TreeNode($"{evnt.name} : {(evnt.end - evnt.start) * 1000f:.00} ms###{context.id}_{i}")) {
                            show_depth = evnt.depth + 1;
                        }
                        ImGui.Indent(ImGui.GetTreeNodeToLabelSpacing());
                    } else {
                        ImGui.Text(string.Format("{0} : {1:.00} ms", evnt.name, (evnt.end - evnt.start) * 1000f));
                    }
                }
            }
            ImGui.Unindent(ImGui.GetTreeNodeToLabelSpacing());
            while (show_depth > 0) {
                ImGui.TreePop();
                --show_depth;
            }
        }

        void DisplayInternal() {
            if (window_visible) {
                last_display_start = curr_context.stopwatch.Elapsed.TotalSeconds;
                while(curr_context.curr_depth>0){
                    End();
                }
                ImGui.SetNextWindowSize(new Vector2(380, 400), ImGuiCond.FirstUseEver);
                ImGui.SetNextWindowPos(new Vector2(70, 70), ImGuiCond.FirstUseEver);
                ImGuiWindowFlags flags = ImGuiWindowFlags.None;
                if(!window_interactive){
                    flags |= ImGuiWindowFlags.NoTitleBar | ImGuiWindowFlags.NoResize;
                    ImGui.SetNextWindowBgAlpha(0f);
                    ImGui.PushStyleVar(ImGuiStyleVar.WindowBorderSize, 0f);
                }
                if (ImGui.Begin("Profiler", ref window_visible, flags)) {
                    if(!UnityEngine.Profiling.Profiler.supported){
                        ImGui.TextColored(new Vector4(1,0,0,1), "Built-in Unity profiler not supported in this build");
                    } else {
                        UnityEngine.Profiling.Profiler.logFile = string.Format("{0}/profile_data", Application.persistentDataPath);
                        bool val = UnityEngine.Profiling.Profiler.enabled;
                        if(ImGui.Checkbox("Recording Unity Profiler", ref val)){
                                UnityEngine.Profiling.Profiler.enableBinaryLog = val;
                                UnityEngine.Profiling.Profiler.enabled = val;      
                        }
                        if(Application.platform == RuntimePlatform.WindowsEditor || Application.platform == RuntimePlatform.WindowsPlayer) {
                            if(System.IO.File.Exists(UnityEngine.Profiling.Profiler.logFile)){
                                ImGui.SameLine();
                                if(ImGui.Button("Show file")){
                                    ShowExplorer(UnityEngine.Profiling.Profiler.logFile);
                                }
                            }
                        }
                    }
                    ImGui.Separator();

                    DisplayProfilerContext(curr_context);
                    if(special_context != null){
                        DisplayProfilerContext(special_context);
                    }
                }
                ImGui.End();
                if (!window_interactive) {
                    ImGui.PopStyleVar(1);
                }
                last_display_end = curr_context.stopwatch.Elapsed.TotalSeconds;
            }
        }
    }
}