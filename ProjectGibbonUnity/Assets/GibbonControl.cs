using ImGuiNET;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;

namespace Wolfire {
public class GibbonControl : MonoBehaviour
{
    public GameObject gibbon;
    public GameObject display_gibbon;
    
    class HandState {
        public Vector3 pos;
        public bool gripping;
    }
    float last_x;

    HandState[] hands;
    int next_hand = 1;
    const float arm_length = 0.8f;
    Character character = new Character();

    class Character {        
        public Transform root;
        public Transform hip;
        public Transform belly;
        public Transform chest;
        public Transform neck;
        public Transform head;
        public Transform leg_root;
        public Transform left_thigh;
        public Transform left_knee;
        public Transform left_foot;
        public Transform right_thigh;
        public Transform right_knee;
        public Transform right_foot;
        public Transform arm_root;
        public Transform left_clavicle;
        public Transform left_shoulder;
        public Transform left_elbow;
        public Transform left_wrist;
        public Transform right_clavicle;
        public Transform right_shoulder;
        public Transform right_elbow;
        public Transform right_wrist;

        public void GetTransforms(Transform p_root){
            root = p_root;
            hip = root.Find("DEF-spine");
            belly = hip.Find("DEF-spine_001");
            chest = belly.Find("DEF-spine_002/DEF-spine_003");
            neck = chest.Find("DEF-spine_004/DEF-spine_005");
            head = neck.Find("DEF-spine_006");
            leg_root = root.Find("torso/MCH-spine_001/MCH-spine/tweak_spine/ORG-spine");
            left_thigh = leg_root.Find("DEF-thigh_L");
            left_knee = left_thigh.Find("DEF-thigh_L_001/DEF-shin_L");
            left_foot = left_knee.Find("DEF-shin_L_001/DEF-foot_L");
            right_thigh = leg_root.Find("DEF-thigh_R");
            right_knee = right_thigh.Find("DEF-thigh_R_001/DEF-shin_R");
            right_foot = right_knee.Find("DEF-shin_R_001/DEF-foot_R");
            arm_root = root.Find("torso/MCH-spine_002/MCH-spine_003/tweak_spine_003/ORG-spine_003");
            left_clavicle = arm_root.Find("ORG-shoulder_L");
            left_shoulder = left_clavicle.Find("DEF-upper_arm_L");
            left_elbow = left_shoulder.Find("DEF-upper_arm_L_001/DEF-forearm_L");
            left_wrist = left_elbow.Find("DEF-forearm_L_001/DEF-hand_L");
            right_clavicle = arm_root.Find("ORG-shoulder_R");
            right_shoulder = right_clavicle.Find("DEF-upper_arm_R");
            right_elbow = right_shoulder.Find("DEF-upper_arm_R_001/DEF-forearm_R");
            right_wrist = right_elbow.Find("DEF-forearm_R_001/DEF-hand_R");
        }

        public void Draw() {
            DebugDraw.Line(neck.position, head.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
            DebugDraw.Line(neck.position, chest.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
            DebugDraw.Line(belly.position, chest.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
            DebugDraw.Line(belly.position, hip.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);

            DebugDraw.Line(left_thigh.position, hip.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
            DebugDraw.Line(left_thigh.position, left_knee.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
            DebugDraw.Line(left_foot.position, left_knee.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
            DebugDraw.Line(left_foot.position, left_foot.TransformPoint(Vector3.right * -0.7f), Color.yellow, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
            
            DebugDraw.Line(right_thigh.position, hip.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
            DebugDraw.Line(right_thigh.position, right_knee.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
            DebugDraw.Line(right_foot.position, right_knee.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
            DebugDraw.Line(right_foot.position, right_foot.TransformPoint(Vector3.right * -0.7f), Color.yellow, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
            
            DebugDraw.Line(left_clavicle.position, chest.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
            DebugDraw.Line(left_clavicle.position, left_shoulder.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
            DebugDraw.Line(left_elbow.position, left_shoulder.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
            DebugDraw.Line(left_elbow.position, left_wrist.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
            DebugDraw.Line(left_wrist.position, left_wrist.TransformPoint(Vector3.right*-1.0f), Color.yellow, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
        
            DebugDraw.Line(right_clavicle.position, chest.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
            DebugDraw.Line(right_clavicle.position, right_shoulder.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
            DebugDraw.Line(right_elbow.position, right_shoulder.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
            DebugDraw.Line(right_elbow.position, right_wrist.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
            DebugDraw.Line(right_wrist.position, right_wrist.TransformPoint(Vector3.right*-1.0f), Color.yellow, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
        }
    }

    float measured_arm_length;

    class VerletPoint {
        public float3 bind_pos;
        public float3 pos;
        public float3 old_pos;
        public float3 temp;
        public float mass;
        public bool pinned;
        public string name;
    }

    class VerletBone {
        public int2 points;
        public float2 length;
        public string name;
    }

    class BindPart {
        public Transform transform;
        public float4x4 bind_mat;
        public quaternion bind_rot;
        public float3 bind_pos;
    }

    BindPart chest = new BindPart();
    BindPart arm_l = new BindPart();
    BindPart arm_r = new BindPart();

    class VerletSystem {
        public List<VerletPoint> points = new List<VerletPoint>();
        public List<VerletBone> bones = new List<VerletBone>();
        
        public void AddPoint(float3 pos, string name){
            var point = new VerletPoint();
            point.bind_pos = pos;
            point.pos = pos;
            point.old_pos = pos;
            point.pinned = false;
            point.name = name;
            point.mass = 1.0f;
            points.Add(point);
        }
    
        public void AddBone(string name, int a, int b) {
            var bone = new VerletBone();
            bone.points[0] = a;
            bone.points[1] = b;
            bone.length = math.distance(points[b].pos, points[a].pos);
            bone.name = name;
            bones.Add(bone);
        }

        public void DrawBones(Color color){
            for(int i=0; i<bones.Count; ++i){
                DebugDraw.Line(points[bones[i].points[0]].pos,points[bones[i].points[1]].pos, color, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);
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

    VerletSystem pendulum = new VerletSystem();
    VerletSystem arms = new VerletSystem();

    void SetBindPart(BindPart part, Transform transform){
        part.transform = transform;
        part.bind_mat = transform.localToWorldMatrix;
        part.bind_pos = transform.position;
        part.bind_rot = transform.rotation;
    }

    // Start is called before the first frame update
    void Start() {
        var pos = gibbon.transform.position;
        pos[1] = 0f;
        pos[2] = 0f;
        hands = new HandState[2];
        for(int i=0; i<2; ++i){
            hands[i] = new HandState();
            hands[i].pos = pos;
            hands[i].gripping = true;
        }
        hands[1].gripping = false;

        pendulum_center = pos;
        //pendulum_length = 
        
        //gibbon.GetComponent<Animator>().runtimeAnimatorController.animationClips[3].SampleAnimation(gibbon, 0.0f);
        //character.GetTransforms(gibbon.transform.Find("rig/root"));
        //character.Draw();            
        
        var root = GameObject.Find("points").transform;
        var neck = root.Find("neck");
        var head = root.Find("head");
        var shoulder = root.Find("shoulder");
        var elbow = root.Find("elbow");
        var grip = root.Find("grip");
        var hip = root.Find("hip");
        var knee = root.Find("knee");
        var foot = root.Find("foot");
        
        //DebugDraw.Line(neck.position, shoulder.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
        //DebugDraw.Line(shoulder.position, elbow.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
        //DebugDraw.Line(elbow.position, grip.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);

        measured_arm_length = Vector3.Distance(shoulder.position, elbow.position) + Vector3.Distance(shoulder.position, elbow.position);
                  
        arms.AddPoint((float3)shoulder.position, "shoulder_r");
        arms.AddPoint((float3)grip.position, "hand_r");
        arms.AddPoint((float3)(shoulder.position+Vector3.right * (neck.position[0] - shoulder.position[0])*2f), "shoulder_l");
        arms.AddPoint((float3)(grip.position+Vector3.right * (neck.position[0] - grip.position[0])*2f), "hand_l");
        arms.AddPoint(new float3(neck.position[0], hip.position[1], neck.position[2]), "body");
        arms.points[0].mass = 2f;
        arms.points[2].mass = 2f;
        arms.points[4].mass = 4f;
        
        arms.AddBone("arm_r", 0, 1);
        arms.bones[arms.bones.Count-1].length[0] *= 0.25f; // Allow arm to flex
        arms.AddBone("arm_l", 2, 3);
        arms.bones[arms.bones.Count-1].length[0] *= 0.25f;
        arms.AddBone("tri_top", 0, 2);
        arms.AddBone("tri_r", 0, 4);
        arms.AddBone("tri_l", 2, 4);

        pendulum.AddPoint(new float3(pendulum_center), "pendulum_axis");
        pendulum.AddPoint(new float3(pendulum_center + new float3(0, -pendulum_length, 0)), "pendulum_end");
        pendulum.AddPoint(new float3(pendulum_center), "pendulum_next_grip");
        pendulum.points[0].pinned = true;
        pendulum.points[2].pinned = true;
        pendulum.AddBone("pendulum", 0, 1);
        pendulum.AddBone("pendulum_next", 2, 1);

        SetBindPart(chest, display_gibbon.transform.Find("DEF-spine_003"));
        SetBindPart(arm_l, display_gibbon.transform.Find("DEF-upper_arm_L"));
        SetBindPart(arm_r, display_gibbon.transform.Find("DEF-upper_arm_R"));
    }
    

        static void DrawCircle(Vector3 pos, float radius){
        int num_segments = 32;
        for(int i=1; i<num_segments+1; ++i){
            float interp = (i-1)/(float)num_segments * Mathf.PI * 2f;
            float interp2 = i/(float)num_segments * Mathf.PI * 2f;
            var temp_pos = pos + (Vector3.right * Mathf.Sin(interp) - Vector3.up * Mathf.Cos(interp))*radius;
            var temp_pos2 = pos + (Vector3.right * Mathf.Sin(interp2) - Vector3.up * Mathf.Cos(interp2))*radius;
            DebugDraw.Line(temp_pos, temp_pos2, new Color(1.0f, 0.0f, 0.0f, 1.0f), DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Normal);
        }
    }

    Vector3 clicked_point;
    float3 com;

    float3 pendulum_center;
    float pendulum_length = 1f;

    // Update is called once per frame
    void Update()
    {
        // Gibbon top speed brachiation is about 15 m/s
        // Can leap up to 8 meters
        // Gibbon wrist is ball and socket
        // Weigh ~7kg, 90 cm tall
        // Gibbons much slower on legs than trees
        // Leg jump speed 8.3 m/s (most force from swinging arms)
        // Ground speed up to 4 m/s?
        // Continous contact can go up to 4 m/s, ricochetal can go as low as 2.5 m/s
        // Ricochetal contact time ~0.5s at 3 m/s, 0.25s at 6 m/s
        // Continuous contact optimized when spaced slightly closer than full arm spread of animal, so around 1.2 m
        // Usually has margin of error, swings with arm not fully extended at high speed
                
        /*
        var editor = GetComponent<AnimationEditor>();
        foreach(var pose in editor.poses){
            if(pose.name == "Hang_pole"){
                AnimationEditor.ApplyPose(gibbon.transform, pose);                
            }
        }*/
        
        var bind_mid = (arms.points[0].bind_pos + arms.points[2].bind_pos + arms.points[4].bind_pos)/3.0f;
        var mid = (arms.points[0].pos + arms.points[2].pos + arms.points[4].pos)/3.0f;
        var forward = math.normalize(math.cross(arms.points[0].pos - arms.points[2].pos, arms.points[0].pos - arms.points[4].pos));
        var up = math.normalize((arms.points[0].pos + arms.points[2].pos)/2.0f - arms.points[4].pos);
        
        //DebugDraw.Line(mid, mid+forward, Color.blue, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);
        //DebugDraw.Line(mid, mid+up, Color.green, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);

        chest.transform.rotation = Quaternion.LookRotation(forward, up) * chest.bind_rot;
        chest.transform.position = mid + (float3)(chest.transform.rotation * (chest.bind_pos - bind_mid));

        arm_l.transform.position = arm_l.bind_pos + (arms.points[2].pos - arms.points[2].bind_pos);
        arm_l.transform.rotation = Quaternion.LookRotation(arms.points[3].pos - arms.points[2].pos, forward) * Quaternion.Inverse(Quaternion.LookRotation(arms.points[3].bind_pos - arms.points[2].bind_pos, Vector3.forward)) * arm_l.bind_rot;
       
        arm_r.transform.position = arm_r.bind_pos + (arms.points[0].pos - arms.points[0].bind_pos);
        arm_r.transform.rotation = Quaternion.LookRotation(arms.points[1].pos - arms.points[0].pos, forward) * Quaternion.Inverse(Quaternion.LookRotation(arms.points[1].bind_pos - arms.points[0].bind_pos, Vector3.forward)) * arm_r.bind_rot;

        //gibbon.transform.position = mid;
        //gibbon.transform.rotation = Quaternion.LookRotation(forward, up);
        
        arms.DrawBones(Color.white);
        pendulum.DrawBones(Color.blue);
        

        float total_mass = 0f;
        var old_com = com;
        com = float3.zero;
        for(int i=0; i<arms.points.Count; ++i){
            com += arms.points[i].pos * arms.points[i].mass;
            total_mass += arms.points[i].mass;
        }
        com /= total_mass;
        DebugDraw.Sphere(com, Color.green, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray );
        //DebugDraw.Line(old_com, com, Color.green, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray );
        
        if(ImGui.Begin("Gibbon")){
            //ImGui.Text($"pendulum_length: {math.distance(arms.points[1].pos, com)}");
            //DebugDraw.Line(arms.points[1].pos, com, Color.green, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray );    
            float3 vel = (pendulum.points[1].pos - pendulum.points[1].old_pos) / (Time.fixedDeltaTime * 0.1f);
            float kinetic_energy = math.lengthsq(vel) / 2.0f;
            ImGui.Text($"kinetic energy: {kinetic_energy}");   
            float height = pendulum.points[1].pos[1] - (pendulum.points[0].pos[1] - pendulum.bones[0].length[1]);
            float potential_energy = height*-Physics.gravity[1];
            ImGui.Text($"potential energy: {potential_energy}");   
            ImGui.Text($"total energy: {kinetic_energy + potential_energy}");   
        }
        ImGui.End();

        if(Input.GetKeyDown(KeyCode.Tab)){
            Time.timeScale = (Time.timeScale == 1.0f)?0.1f:1.0f;
        }

        const bool pendulum_test = false;
        if(pendulum_test){
            float pendulum_period = math.PI * 2.0f * math.sqrt(pendulum_length / -Physics.gravity[1]);
            float pendulum_angle = math.sin(math.PI * 2.0f / pendulum_period * Time.time);
            var pendulum_pos = new float3(math.sin(pendulum_angle), -math.cos(pendulum_angle), 0f) * pendulum_length + pendulum_center;
            DebugDraw.Sphere(pendulum_pos, Color.blue, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray );
        
            DebugDraw.Line(pendulum_center, pendulum_pos, Color.blue, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray );    
        }
    }
    
    float grabbed_time = 0f;

    void Step(float step) {
        float horz_input = 0f;
        float vert_input = 0f;
        if(Input.GetKey(KeyCode.D)){
            horz_input = 1f;
        }
        if(Input.GetKey(KeyCode.A)){
            horz_input = -1f;
        }
        if(Input.GetKey(KeyCode.W)){
            vert_input = 1f;
        }
        if(Input.GetKey(KeyCode.S)){
            vert_input = -1f;
        }
        
        for(int i=0; i<10; ++i){
            float temp_step = step * 0.1f;
            float time_sqrd = temp_step * temp_step;
            pendulum.StartSim(temp_step);
            pendulum.points[1].pos[0] += horz_input * time_sqrd * 20f;
            pendulum.Constraints();
            pendulum.EndSim();
            if(vert_input != 0.0f){
                pendulum.bones[0].length[0] -= vert_input * temp_step;
                pendulum.bones[0].length[1] -= vert_input * temp_step;
                pendulum.bones[1].length[0] -= vert_input * temp_step;
                pendulum.bones[1].length[1] -= vert_input * temp_step;
                pendulum.Constraints();
            }
        }

        bool arms_map = true;
        if(arms_map){
            arms.points[1].pos = pendulum.points[0].pos;
            arms.points[1].pinned = true;
            arms.StartSim(step);
            for(int j=0; j<1; ++j){
                float total_mass = 0f;
                var com = float3.zero;
                for(int i=0; i<arms.points.Count; ++i){
                    if(arms.points[i].pinned == false){
                        com += arms.points[i].pos * arms.points[i].mass;
                        total_mass += arms.points[i].mass;
                    }
                }
                com /= total_mass;
                var offset = pendulum.points[1].pos - com;
                arms.points[0].pos += offset * total_mass / arms.points[0].mass;
                /*for(int i=0; i<arms.points.Count; ++i){
                    if(arms.points[i].pinned == false){
                        arms.points[i].pos += offset;
                    }
                }*/
                arms.Constraints();
            }
            arms.EndSim();
        }

        bool arms_sim = false;
        if(arms_sim){        
            float time_sqrd = step * step;
            if(hands[0].gripping){    
                //points[1].pos = hands[0].pos;
                arms.points[1].pinned = true;
            } else {
                arms.points[1].pinned = false;
            }
            if(hands[1].gripping){    
                //points[3].pos = hands[1].pos;
                arms.points[3].pinned = true;
            } else {
                arms.points[3].pinned = false;
            }
        
            arms.StartSim(step);

            const int swing_in_place = 0, continuous = 1, ricochetal = 2; 

            int state = continuous;
            if(horz_input != 0f){
                var swing_force = 3f;
                float offset = horz_input * time_sqrd * swing_force;
                switch(state){
                    case swing_in_place:
                        arms.points[0].pos[0] += offset;
                        arms.points[2].pos[0] += offset;
                        arms.points[4].pos[0] += offset;
                        break;
                    case continuous:
                        if(grabbed_time <= Time.time - 0.2f){
                            hands[next_hand].gripping = false;
                        }
                        arms.points[0].pos[0] += offset;
                        arms.points[2].pos[0] += offset;
                        arms.points[4].pos[0] += offset;
                        var grip_pos = (float3)hands[1-next_hand].pos;
                        grip_pos[0] += arm_length * horz_input * 1.8f;
                        
                        if(grabbed_time <= Time.time - 0.4f){
                            float max_grab_force = 20f; 
                            var offset_len = time_sqrd * max_grab_force;// math.min(1f, d_key_held*1f) * max_grab_force;
                            int hand_point = next_hand*2+1;
                            if(math.distance(arms.points[hand_point].pos, grip_pos) < 0.1f){
                                //points[hand_point].pos = grip_pos;
                                arms.points[hand_point].pinned = true;
                                hands[next_hand].gripping = true;
                                hands[next_hand].pos = grip_pos;
                                next_hand = 1-next_hand;
                                //hands[next_hand].gripping = false;
                                grabbed_time = Time.time;
                            } else {
                                arms.points[hand_point].pos += math.normalize((float3)grip_pos - arms.points[hand_point].pos) * offset_len;
                            }
                        }
                        break;
                    case ricochetal:
                        break;
                }
            } 
            arms.Constraints();
            arms.Constraints();
            arms.Constraints();
            arms.Constraints();
            arms.EndSim();
        }
    }

    private void FixedUpdate() {
        Step(Time.fixedDeltaTime);
    }
}
}