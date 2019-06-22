using ImGuiNET;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;

namespace Wolfire {
public class Verlet {
    public static GameObject point_prefab_static;

    public class Point {
        public float3 bind_pos;
        public float3 pos;
        public float3 old_pos;
        public float3 temp;
        public float mass;
        public bool pinned;
        public string name;
        public Transform widget;
    }

    public class Bone {
        public int2 points;
        public float2 length;
        public string name;
        public bool enabled;
    }
    
    public class System {
        public List<Point> points = new List<Point>();
        public List<Bone> bones = new List<Bone>();
        
        public void AddPoint(float3 pos, string name){
            var point = new Point();
            point.bind_pos = pos;
            point.pos = pos;
            point.old_pos = pos;
            point.pinned = false;
            point.name = name;
            point.mass = 1.0f;
            point.widget = GameObject.Instantiate(point_prefab_static, point.pos, Quaternion.identity).transform;
            points.Add(point);
        }
    
        public void AddBone(string name, int a, int b) {
            var bone = new Bone();
            bone.points[0] = a;
            bone.points[1] = b;
            bone.length = math.distance(points[b].pos, points[a].pos);
            bone.name = name;
            bone.enabled = true;
            bones.Add(bone);
        }

        public void DrawBones(Color color){
            for(int i=0; i<bones.Count; ++i){
                if(bones[i].enabled){
                    DebugDraw.Line(points[bones[i].points[0]].pos,points[bones[i].points[1]].pos, color, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);
                }
            }       
        }

        public void StartSim(float step){
            float time_sqrd = step * step;
            var acc = new float3(0, Physics.gravity[1], 0);
            for(int i=0; i<points.Count; ++i){
                var point = points[i];
                point.temp = point.pos;
                if(!point.pinned){
                    point.pos = point.pos + (point.pos - point.old_pos) +  acc * time_sqrd;
                }
            }
        }

        public void Constraints() {
            for(int i=0; i<bones.Count; ++i){
                if(!bones[i].enabled){
                    continue;
                }
                var bone = bones[i];
                int num_pinned = 0;
                if(points[bone.points[0]].pinned) {++num_pinned;}
                if(points[bone.points[1]].pinned) {++num_pinned;}
                if(num_pinned < 2){
                    float curr_len = math.distance(points[bone.points[0]].pos, points[bone.points[1]].pos);
                    if(curr_len != 0f){
                        if(num_pinned == 1){
                            int pinned = bone.points[1];
                            int unpinned = bone.points[0];
                            if(points[bone.points[0]].pinned){
                                pinned = bone.points[0];
                                unpinned = bone.points[1];
                            }
                            var unpinned_point = points[unpinned];
                            if(curr_len < bone.length[0]){
                                unpinned_point.pos = points[pinned].pos + (points[unpinned].pos - points[pinned].pos) / curr_len * bone.length[0];
                            } else if(curr_len > bone.length[1]){
                                unpinned_point.pos = points[pinned].pos + (points[unpinned].pos - points[pinned].pos) / curr_len * bone.length[1];
                            }
                        } else {
                            var offset = (points[bone.points[1]].pos - points[bone.points[0]].pos) / curr_len;
                            float rel_mass = points[bone.points[1]].mass / (points[bone.points[0]].mass + points[bone.points[1]].mass);
                            var mid = points[bone.points[0]].pos * (1f-rel_mass) + points[bone.points[1]].pos * rel_mass;
                            if(curr_len < bone.length[0]){
                                points[bone.points[0]].pos = mid - offset * bone.length[0] * rel_mass;
                                points[bone.points[1]].pos = mid + offset * bone.length[0] * (1f-rel_mass);
                            } else if(curr_len > bone.length[1]){
                                points[bone.points[0]].pos = mid - offset * bone.length[1] * rel_mass;
                                points[bone.points[1]].pos = mid + offset * bone.length[1] * (1f-rel_mass);
                            }
                        }
                    }
                }
            }
        }

        public void EndSim() {
            for(int i=0; i < points.Count; ++i) {
                var point = points[i];
                point.old_pos = point.temp;
            }
        }

        public void Step(float step) {
            StartSim(step);
            Constraints();
            EndSim();
        }
    }
}
}