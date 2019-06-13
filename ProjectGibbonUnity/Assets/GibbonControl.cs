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
    
    class HandState {
        public Vector3 pos;
        public bool gripping;
    }
    float last_x;

    HandState[] hands;
    int next_hand = 0;
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

    List<VerletPoint> points = new List<VerletPoint>();
    List<VerletBone> bones = new List<VerletBone>();

    void AddPoint(float3 pos, string name){
        var point = new VerletPoint();
        point.pos = pos;
        point.old_pos = pos;
        point.pinned = false;
        point.name = name;
        point.mass = 1.0f;
        points.Add(point);
    }
    
    void AddBone(string name, int a, int b) {
        var bone = new VerletBone();
        bone.points[0] = a;
        bone.points[1] = b;
        bone.length = math.distance(points[b].pos, points[a].pos);
        bone.name = name;
        bones.Add(bone);
    }

    // Start is called before the first frame update
    void Start() {
        var pos = Vector3.zero;
        hands = new HandState[2];
        for(int i=0; i<2; ++i){
            hands[i] = new HandState();
            hands[i].pos = pos;
            hands[i].gripping = true;
        }
        hands[1].gripping = false;
        
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
                  
        AddPoint((float3)shoulder.position, "shoulder_r");
        AddPoint((float3)grip.position, "hand_r");
        AddPoint((float3)(shoulder.position+Vector3.right * (neck.position[0] - shoulder.position[0])*2f), "shoulder_l");
        AddPoint((float3)(grip.position+Vector3.right * (neck.position[0] - grip.position[0])*2f), "hand_l");
        AddPoint(new float3(neck.position[0], hip.position[1], neck.position[2]), "body");
        AddBone("arm_r", 0, 1);
        bones[bones.Count-1].length[0] *= 0.25f; // Allow arm to flex
        AddBone("arm_l", 2, 3);
        bones[bones.Count-1].length[0] *= 0.25f;
        AddBone("tri_top", 0, 2);
        AddBone("tri_r", 0, 4);
        AddBone("tri_l", 2, 4);
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
        if(false){ // Walking on branch
            float max_speed = 4f;
            Vector3 target_vel = Vector3.zero;
            if(Input.GetKey(KeyCode.A)){
                target_vel = -Vector3.right * max_speed;
            }
            if(Input.GetKey(KeyCode.D)){
                target_vel = Vector3.right * max_speed;
            }
            //vel = Vector3.Lerp(target_vel, vel, Mathf.Pow(0.01f, Time.deltaTime));
            //transform.position += vel * Time.deltaTime;
        }
        
        
        /*
        var editor = GetComponent<AnimationEditor>();
        foreach(var pose in editor.poses){
            if(pose.name == "Hang_pole"){
                AnimationEditor.ApplyPose(gibbon.transform, pose);                
            }
        }*/
        
        if(hands[0].gripping){    
            points[1].pos = hands[0].pos;
            points[1].pinned = true;
        } else {
            points[1].pinned = false;
        }
        if(hands[1].gripping){    
            points[3].pos = hands[1].pos;
            points[3].pinned = true;
        } else {
            points[3].pinned = false;
        }


        for(int i=0; i<bones.Count; ++i){
            DebugDraw.Line(points[bones[i].points[0]].pos,points[bones[i].points[1]].pos, Color.white, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);
        }
    }

    private void FixedUpdate() {
        float time_sqrd = Time.fixedDeltaTime * Time.fixedDeltaTime;
        float side_speed = 5.0f;
        float max_speed = 15f;
        var acc = new float3(0, Physics.gravity[1], 0);
        if(Input.GetKey(KeyCode.A)){
            acc -= (float3)Vector3.right * side_speed;
        }
        if(Input.GetKey(KeyCode.D)){
            acc += (float3)Vector3.right * side_speed;
        }

        for(int i=0; i<points.Count; ++i){
            var point = points[i];
            if(!point.pinned){
                point.temp = point.pos;
                point.pos = point.pos + (point.pos - point.old_pos) +  acc * time_sqrd;
            }
        }
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
                        var mid = (points[bone.points[0]].pos + points[bone.points[1]].pos) * 0.5f;
                        var offset = (points[bone.points[1]].pos - points[bone.points[0]].pos) / curr_len;
                        if(curr_len < bone.length[0]){
                            points[bone.points[0]].pos = mid - offset * bone.length[0] * 0.5f;
                            points[bone.points[1]].pos = mid + offset * bone.length[0] * 0.5f;
                        } else if(curr_len > bone.length[1]){
                            points[bone.points[0]].pos = mid - offset * bone.length[1] * 0.5f;
                            points[bone.points[1]].pos = mid + offset * bone.length[1] * 0.5f;
                        }
                    }
                }
            }
        }
        for(int i=0; i<points.Count; ++i){
            var point = points[i];
            if(!point.pinned){
                point.old_pos = point.temp;
            }
        }
    }
}
}