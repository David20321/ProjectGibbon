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
    public GameObject point_prefab;
    static public GameObject point_prefab_static;
    
    class HandState {
        public Vector3 pos;
        public bool gripping;
    }
    float last_x;

    HandState[] hands;
    int next_hand = 1;
    const float arm_length = 0.8f;
    
    float measured_arm_length;

    class VerletPoint {
        public float3 bind_pos;
        public float3 pos;
        public float3 old_pos;
        public float3 temp;
        public float mass;
        public bool pinned;
        public string name;
        public Transform widget;
    }

    class VerletBone {
        public int2 points;
        public float2 length;
        public string name;
        public bool enabled;
    }

    class BindPart {
        public Transform transform;
        public float4x4 bind_mat;
        public quaternion bind_rot;
        public float3 bind_pos;
    }

    BindPart chest = new BindPart();
    BindPart arm_top_l = new BindPart();
    BindPart arm_bottom_l = new BindPart();
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
            point.widget = Instantiate(point_prefab_static, point.pos, Quaternion.identity).transform;
            points.Add(point);
        }
    
        public void AddBone(string name, int a, int b) {
            var bone = new VerletBone();
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

    VerletSystem pendulum = new VerletSystem();
    VerletSystem arms = new VerletSystem();

    void SetBindPart(BindPart part, Transform transform){
        part.transform = transform;
        part.bind_mat = transform.localToWorldMatrix;
        part.bind_pos = transform.position;
        part.bind_rot = transform.rotation;
    }

    // Start is called before the first frame update
    
    class TwoBoneIK {
        public float3[] points;
        public TwoBoneIK() {
            points = new float3[3];
        }
    }

    TwoBoneIK left_arm = new TwoBoneIK();

    Vector3 simple_pos;
    Vector3 simple_vel = Vector3.zero;
    void Start() {
        point_prefab_static = point_prefab;

        simple_pos = gibbon.transform.position;
        simple_pos[1] = 0f;
        simple_pos[2] = 0f;
        hands = new HandState[2];
        for(int i=0; i<2; ++i){
            hands[i] = new HandState();
            hands[i].pos = simple_pos;
            hands[i].gripping = true;
        }
        hands[1].gripping = false;

        pendulum_center = simple_pos;
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

        measured_arm_length = Vector3.Distance(shoulder.position, elbow.position) + Vector3.Distance(elbow.position, grip.position);
                  
        left_arm.points[0] = shoulder.position;
        left_arm.points[1] = elbow.position;
        left_arm.points[2] = grip.position;

        arms.AddPoint((float3)shoulder.position, "shoulder_r");
        arms.AddPoint((float3)grip.position, "hand_r");
        arms.AddPoint((float3)(shoulder.position+Vector3.right * (neck.position[0] - shoulder.position[0])*2f), "shoulder_l");
        arms.AddPoint((float3)(grip.position+Vector3.right * (neck.position[0] - grip.position[0])*2f), "hand_l");
        arms.AddPoint(new float3(neck.position[0], hip.position[1], neck.position[2]), "body");
        arms.points[0].mass = 2f;
        arms.points[2].mass = 2f;
        arms.points[4].mass = 4f;
        
        arms.AddBone("arm_r", 0, 1);
        arms.bones[arms.bones.Count-1].length[1] = measured_arm_length;
        arms.bones[arms.bones.Count-1].length[0] *= 0.25f; // Allow arm to flex
        arms.AddBone("arm_l", 2, 3);
        arms.bones[arms.bones.Count-1].length[1] = measured_arm_length;
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
        SetBindPart(arm_top_l, display_gibbon.transform.Find("DEF-upper_arm_L"));
        SetBindPart(arm_bottom_l, display_gibbon.transform.Find("DEF-forearm_L"));
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
    float pendulum_length = 0.9f;

    void GripPoint(float3 pos){
        hands[next_hand].pos = pos;
        hands[next_hand].gripping = true;
        hands[1-next_hand].gripping = false;
        next_hand = 1 - next_hand;
        pendulum.points[0].pos = pos;
        pendulum.points[2].pos = pos;
        pendulum.bones[0].enabled = true;
        pendulum.bones[1].enabled = true;
        float len = math.distance(pendulum.points[0].pos, pendulum.points[1].pos);
        pendulum.bones[0].length[0] = len;
        pendulum.bones[1].length[0] = len;
        pendulum.bones[0].length[1] = len;
        pendulum.bones[1].length[1] = len;
    }
    
    float swing_time = 0f;

    float GetAngleGivenSides(float a, float b, float c){
        // law of cosines:
        // c*c = a*a + b*b - 2*a*b*cos(C)
        // c*c - a*a - b*b = -2*a*b*cos(C)
        // (c*c - a*a - b*b) / (-2*a*b) = cos(C)
        // C = acos((c*c - a*a - b*b) / (-2*a*b))
        var top = (c*c - a*a - b*b);
        var divisor = (-2*a*b);
        if(divisor==0f){
            return 0f;
        }
        return math.acos(math.clamp(top / divisor, -1f, 1f));
    }

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
        // Pendulum from 80 degrees w length 1 m has base speed of 4 m/s
                
        /*
        var editor = GetComponent<AnimationEditor>();
        foreach(var pose in editor.poses){
            if(pose.name == "Hang_pole"){
                AnimationEditor.ApplyPose(gibbon.transform, pose);                
            }
        }*/
        
        const bool manually_drag_points = false;
        if(manually_drag_points){
            for(int i=0; i<arms.points.Count; ++i){
                arms.points[i].pos = arms.points[i].widget.position;
            }
            arms.Constraints();
            for(int i=0; i<arms.points.Count; ++i){
                arms.points[i].widget.position = arms.points[i].pos;
            }
        }

        var bind_mid = (arms.points[0].bind_pos + arms.points[2].bind_pos + arms.points[4].bind_pos)/3.0f;
        var mid = (arms.points[0].pos + arms.points[2].pos + arms.points[4].pos)/3.0f;
        var forward = -math.normalize(math.cross(arms.points[0].pos - arms.points[2].pos, arms.points[0].pos - arms.points[4].pos));
        var bind_forward = -math.normalize(math.cross(arms.points[0].bind_pos - arms.points[2].bind_pos, arms.points[0].bind_pos - arms.points[4].bind_pos));
        var up = math.normalize((arms.points[0].pos + arms.points[2].pos)/2.0f - arms.points[4].pos);
        var bind_up = math.normalize((arms.points[0].bind_pos + arms.points[2].bind_pos)/2.0f - arms.points[4].bind_pos);
        
        //DebugDraw.Line(mid, mid+forward, Color.blue, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);
        //DebugDraw.Line(mid, mid+up, Color.green, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);

        chest.transform.rotation = Quaternion.LookRotation(forward, up) * Quaternion.Inverse(Quaternion.LookRotation(bind_forward, bind_up)) * chest.bind_rot;
        chest.transform.position = mid + (float3)(Quaternion.LookRotation(forward, up) * Quaternion.Inverse(Quaternion.LookRotation(bind_forward, bind_up)) * (chest.bind_pos - bind_mid));

        var shoulder_offset = (arms.points[2].pos - arms.points[2].bind_pos);
        var shoulder_rotation = Quaternion.LookRotation(arms.points[3].pos - arms.points[2].pos, forward) * Quaternion.Inverse(Quaternion.LookRotation(arms.points[3].bind_pos - arms.points[2].bind_pos, Vector3.forward));
        
        float dist_a = math.distance(left_arm.points[0], left_arm.points[1]);
        float dist_b = math.distance(left_arm.points[1], left_arm.points[2]);
        float dist_c = math.distance(arms.points[2].pos, arms.points[3].pos);
        var elbow_angle = GetAngleGivenSides(dist_a, dist_b, dist_c);
        var shoulder_angle = GetAngleGivenSides(dist_c, dist_a, dist_b);
        DebugText.AddVar("elbow_angle", elbow_angle, 0.5f);
        DebugText.AddVar("shoulder_angle", shoulder_angle, 0.5f);

        var elbow_axis = shoulder_rotation * Vector3.forward;// math.normalize(shoulder_rotation * math.cross(left_arm.points[2] - left_arm.points[1], left_arm.points[1] - left_arm.points[0]));
        shoulder_rotation = Quaternion.AngleAxis(shoulder_angle * Mathf.Rad2Deg, elbow_axis) * shoulder_rotation;
        arm_top_l.transform.position = arm_top_l.bind_pos + shoulder_offset;
        arm_top_l.transform.rotation = shoulder_rotation * arm_top_l.bind_rot;
        
        var elbow = arm_top_l.transform.position + arm_top_l.transform.rotation * Quaternion.Inverse(arm_top_l.bind_rot) * (arm_bottom_l.bind_pos - arm_top_l.bind_pos);
        arm_bottom_l.transform.position = elbow;
        arm_bottom_l.transform.rotation = Quaternion.AngleAxis((math.PI + elbow_angle) * Mathf.Rad2Deg, elbow_axis) * shoulder_rotation * arm_bottom_l.bind_rot;

        //DebugDraw.Line(left_arm.points[0], left_arm.points[1], Color.green, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);
        //DebugDraw.Line(left_arm.points[1], left_arm.points[2], Color.blue, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);


        arm_r.transform.position = arm_r.bind_pos + (arms.points[0].pos - arms.points[0].bind_pos);
        arm_r.transform.rotation = Quaternion.LookRotation(arms.points[1].pos - arms.points[0].pos, forward) * Quaternion.Inverse(Quaternion.LookRotation(arms.points[1].bind_pos - arms.points[0].bind_pos, Vector3.forward)) * arm_r.bind_rot;

        //gibbon.transform.position = mid;
        //gibbon.transform.rotation = Quaternion.LookRotation(forward, up);
        
        arms.DrawBones(Color.white);
        pendulum.DrawBones(Color.blue);
        
        if(!hands[0].gripping && !hands[1].gripping){
            var vel = (pendulum.points[1].pos - pendulum.points[1].old_pos) / (Time.fixedDeltaTime * 0.1f);
            var perp = new float3(vel[1], -vel[0], 0f);
            if(perp[1] < 0){
                perp = -perp;
            }
            perp = math.normalize(perp);
            float len = -pendulum.points[1].pos[1] / perp[1];
            var point = pendulum.points[1].pos + perp * len;
            DebugDraw.Line(pendulum.points[1].pos, point, Color.red, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);
        }

        if(Input.GetKeyDown(KeyCode.Space)){
            if(hands[0].gripping || hands[1].gripping ){
                hands[0].gripping = false;
                hands[1].gripping = false;
                pendulum.bones[0].enabled = false;
                pendulum.bones[1].enabled = false;
            } else {
                var vel = (pendulum.points[1].pos - pendulum.points[1].old_pos) / (Time.fixedDeltaTime * 0.1f);
                var perp = new float3(vel[1], -vel[0], 0f);
                if(perp[1] < 0){
                    perp = -perp;
                }
                perp = math.normalize(perp);
                float len = -pendulum.points[1].pos[1] / perp[1];
                /*if(len > arm_length){
                    len = arm_length;
                }*/
                var point = pendulum.points[1].pos + perp * len;
                point[1] = 0f;
                GripPoint(point);
            }
        }

        float total_mass = 0f;
        var old_com = com;
        com = float3.zero;
        for(int i=0; i<arms.points.Count; ++i){
            com += arms.points[i].pos * arms.points[i].mass;
            total_mass += arms.points[i].mass;
        }
        com /= total_mass;
        DebugDraw.Sphere(com, Color.green, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray );
                
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
            ImGui.Text($"horz speed: {math.abs(vel[0])}");  
            ImGui.Text($"horz speed: {math.abs(simple_vel[0])}");  
            ImGui.Text($"swing time: {swing_time}");  
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

        DebugDraw.Sphere(pendulum.points[1].pos, Color.blue, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray );
        
    }
    
    float grabbed_time = 0f;

    float3 MoveTowards(float3 a, float3 b, float max_dist){
        float len = math.distance(a,b);
        if(len < max_dist){
            return b;
        } else {
            return a + (b-a)/len*max_dist;
        }
    }

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

        
        var old_pos = simple_pos;
        simple_vel[0] += horz_input * Time.deltaTime * 5f;
        simple_vel[0] = math.clamp(simple_vel[0], -15f, 15f);
        simple_pos += simple_vel * Time.deltaTime;
        float amplitude = math.pow(math.abs(simple_vel[0])/10f + 1f, 0.8f)-1f;
        float min_height = -1f + amplitude * 0.5f;
        var old_swing_time = swing_time;
        swing_time = Time.time*8f/(math.PI*2f);
        if(math.ceil(old_swing_time) != math.ceil(swing_time)){
            next_hand = 1-next_hand;
        }
        simple_pos[1] = (min_height + (math.sin((swing_time-0.1f) * (math.PI*2f))+1f)*amplitude) * pendulum_length;
        DebugDraw.Sphere(simple_pos, Color.green, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray );
        //DebugDraw.Line(old_pos, simple_pos, Color.green, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray );    
        float next_trough_time = ((math.ceil(swing_time)-0.25f) * (math.PI * 2.0f))/8f;
        hands[next_hand].pos = simple_pos + simple_vel * (next_trough_time-Time.time);
        hands[next_hand].pos[1] = 0.0f;
        DebugDraw.Sphere(hands[0].pos, Color.green, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray );
        DebugDraw.Sphere(hands[1].pos, Color.blue, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray );
        
        
        // If movement key is held, then look at trajectory
        // If we have a good jump trajectory, then let go and follow trajectory and grab at best grab point
        // Otherwise if we're at the end of our swing and can reach the next handhold, then start a new swing (going backwards slightly for momentum)
        // Otherwise swing more to get momentum
        
        bool check_for_jump = false;
        if(check_for_jump){
            if(horz_input != 0.0f && (hands[0].gripping || hands[1].gripping)) {
                var temp_pos = pendulum.points[1].pos;
                float3 vel = (pendulum.points[1].pos - pendulum.points[1].old_pos) / (Time.fixedDeltaTime * 0.1f);
                for (int i = 0; i < 30; ++i) {
                    temp_pos += vel * Time.fixedDeltaTime;
                    DebugDraw.Sphere(temp_pos, Color.red, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray);

                    var perp = new float3(vel[1], -vel[0], 0f);
                    if (perp[1] < 0) {
                        perp = -perp;
                    }
                    bool any_good_trajectory = false;
                    if (perp[1] != 0) {
                        perp = math.normalize(perp);
                        float len = -temp_pos[1] / perp[1];
                        var point = temp_pos + perp * len;
                        var color = Color.red;
                        float dist = math.distance(point, temp_pos);
                        // Highlight grip if it will be within reach and will be going in the same direction
                        if (dist < 1.0f && dist > 0.75f && perp[0] * vel[0] > 0.0f && math.abs(point[0] - hands[1-next_hand].pos[0]) > arm_length * 2f) {
                            color = Color.green;
                            any_good_trajectory = true;
                        }
                        DebugDraw.Line(temp_pos, point, color, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray);
                        if(any_good_trajectory){
                            // let go
                            hands[0].gripping = false;
                            hands[1].gripping = false;
                            pendulum.bones[0].enabled = false;
                            pendulum.bones[1].enabled = false;
                            Debug.Log("LET GO");
                            break;
                        }
                    }

                    vel += (float3)Physics.gravity * 0.1f;
                }
            }
        }

        bool use_pendulum = false;
        if(use_pendulum){
            for (int i=0; i<10; ++i){
                if(!hands[0].gripping && !hands[1].gripping){
                    float3 vel = (pendulum.points[1].pos - pendulum.points[1].old_pos) / (Time.fixedDeltaTime * 0.1f);
                    var temp_pos = pendulum.points[1].pos;

                    var perp = new float3(vel[1], -vel[0], 0f);
                    if (perp[1] < 0) {
                        perp = -perp;
                    }
                    if (perp[1] != 0) {
                        perp = math.normalize(perp);
                        float len = -temp_pos[1] / perp[1];
                        var point = temp_pos + perp * len;
                        var color = Color.red;
                        float dist = math.distance(point, temp_pos);
                        // Highlight grip if it will be within reach and will be going in the same direction
                        if (dist < 1.0f && dist > 0.75f && perp[0] * vel[0] > 0.0f && math.abs(point[0] - hands[1-next_hand].pos[0]) > arm_length * 2f) {
                            color = Color.green;
                            GripPoint(point);
                            Debug.Log("GRAB");
                            break;
                        }
                        DebugDraw.Line(temp_pos, point, color, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray);
                    }
                }

            float temp_step = step * 0.1f;
            float time_sqrd = temp_step * temp_step;
            pendulum.StartSim(temp_step);
            pendulum.points[1].pos[0] += horz_input * time_sqrd * 2f;
            pendulum.Constraints();
            pendulum.EndSim();

            /*
            if(horz_input != 0.0f){
                var grab_point = (float3)hands[1-next_hand].pos + new float3(arm_length * 1.8f * horz_input,0,0);
                if(math.distance(pendulum.points[1].pos, grab_point) < arm_length){        
                    GripPoint(grab_point);
                }
            }*/
            
                for(int a=0; a<2; ++a){
                    for(int b=0; b<2; ++b){
                        pendulum.bones[a].length[b] = Mathf.MoveTowards(pendulum.bones[a].length[b], pendulum_length, temp_step);
                    }
                }
                pendulum.Constraints();

            /*
            if(vert_input != 0.0f){
                pendulum.bones[0].length[0] -= vert_input * temp_step;
                pendulum.bones[0].length[1] -= vert_input * temp_step;
                pendulum.bones[1].length[0] -= vert_input * temp_step;
                pendulum.bones[1].length[1] -= vert_input * temp_step;
                pendulum.Constraints();
            }*/

            const bool draw_com_path = false;
            if(draw_com_path){
                DebugDraw.Line(pendulum.points[1].old_pos, pendulum.points[1].pos, Color.green, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray );
            }
        }
        }

        bool arms_map = true;
        if(arms_map){
            /*arms.points[1].pinned = hands[0].gripping;
            if(hands[0].gripping){
                arms.points[1].pos = pendulum.points[0].pos;
            } else {
                arms.points[1].pos = arms.points[0].pos + new float3(measured_arm_length,0,0);
            }
            arms.points[3].pinned = hands[1].gripping;
            if(hands[1].gripping){
                arms.points[3].pos = pendulum.points[2].pos;
            } else {
                arms.points[3].pos = arms.points[2].pos + new float3(measured_arm_length,0,0);
            }*/
            
            for(int i=0; i<2; ++i){
               // if((hands[i].pos[0] - simple_pos[0]) * simple_vel[0] > 0f){
                    arms.points[i*2+1].pos = MoveTowards(arms.points[i*2+1].pos, hands[i].pos, math.max(0f, math.pow(math.cos((swing_time+0.25f+(1-i))*math.PI*1f)*0.5f+0.5f, 1f)) * step * 5f);
                //}   
            }
            arms.StartSim(step);
            for(int j=0; j<4; ++j){
                float total_mass = 0f;
                var com = float3.zero;
                for(int i=0; i<arms.points.Count; ++i){
                    if(arms.points[i].pinned == false){
                        com += arms.points[i].pos * arms.points[i].mass;
                        total_mass += arms.points[i].mass;
                    }
                }
                com /= total_mass;
                var offset = (float3)simple_pos - com;
                /*if(arms.points[1].pinned && !arms.points[3].pinned){
                    arms.points[0].pos += offset * total_mass / arms.points[0].mass;
                } else if(arms.points[3].pinned && !arms.points[1].pinned){
                    arms.points[2].pos += offset * total_mass / arms.points[2].mass;
                } else {
                    var temp_offset = offset * total_mass / (arms.points[0].mass + arms.points[2].mass);
                    arms.points[0].pos += temp_offset;
                    arms.points[2].pos += temp_offset;
                }*/
                for(int i=0; i<arms.points.Count; ++i){
                    if(i!=1 && i!=3){
                        arms.points[i].pos += offset;
                    }
                }
                // Apply torque to keep torso upright
                float step_sqrd = step*step;
                float force = 20f;
                arms.points[4].pos[1] -= step_sqrd * force;
                arms.points[0].pos[1] += step_sqrd * force * 0.5f;
                arms.points[2].pos[1] += step_sqrd * force * 0.5f;
                arms.points[0].pos[2] -= step_sqrd * simple_vel[0];
                arms.points[2].pos[2] += step_sqrd * simple_vel[0];
                arms.points[4].pos[0] -= simple_vel[0] * step_sqrd * 2f; // Apply backwards force to maintain forwards tilt
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
        var cam_pos = Camera.main.transform.position;
        cam_pos[0] = simple_pos[0];
        Camera.main.transform.position = cam_pos;
    }

    private void FixedUpdate() {
        Step(Time.fixedDeltaTime);
    }
}
}