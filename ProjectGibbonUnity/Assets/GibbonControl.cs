using ImGuiNET;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;

namespace Wolfire {
public class GibbonControl : MonoBehaviour {
    public GameObject display_gibbon; // Character mesh and bone transforms
    public GameObject point_prefab; // Widget for manipulating point positions
    
    // Current target position for each hand
    float3[] limb_targets;
    int next_hand = 1; // Which hand to grip with next
    float3 locked_handhold;
    
    // Bind poses for each skinned bone
    class BindPart {
        public Transform transform;
        public float4x4 bind_mat;
        public quaternion bind_rot;
        public float3 bind_pos;
    }

    void SetBindPart(BindPart part, Transform transform){
        part.transform = transform;
        part.bind_mat = transform.localToWorldMatrix;
        part.bind_pos = transform.position;
        part.bind_rot = transform.rotation;
    }

    class BindParts {
        public BindPart chest = new BindPart();
        public BindPart arm_top_l = new BindPart();
        public BindPart arm_bottom_l = new BindPart();
        public BindPart arm_top_r = new BindPart();
        public BindPart arm_bottom_r = new BindPart();
        public BindPart head = new BindPart();
        public BindPart belly = new BindPart();
        public BindPart pelvis = new BindPart();
        public BindPart leg_top_l = new BindPart();
        public BindPart leg_bottom_l = new BindPart();
        public BindPart leg_top_r = new BindPart();
        public BindPart leg_bottom_r = new BindPart();
    }

    BindParts bind_parts = new BindParts();
    
    // Particle simulation systems
    Verlet.System arms =     new Verlet.System();
    Verlet.System complete = new Verlet.System();
    Verlet.System branches = new Verlet.System();

    // IK helper
    class TwoBoneIK {
        public float3[] points;
        public TwoBoneIK() {
            points = new float3[3];
        }
    }

    TwoBoneIK arm_ik = new TwoBoneIK();
    TwoBoneIK leg_ik = new TwoBoneIK();

    // Simple character particle information
    float3 simple_pos;
    float3 simple_vel = float3.zero;
    float swing_time = 0f;
    float walk_time = 0f;

    class MovementSystem {
        public float3 target_com;
        public float3[] limb_targets = new float3[4];
        public Verlet.System arms = new Verlet.System();
    }
    MovementSystem walk = new MovementSystem();
    MovementSystem swing = new MovementSystem();
    MovementSystem jump = new MovementSystem();

    void Start() {
        Verlet.point_prefab_static = point_prefab;

        // Starting point
        simple_pos = display_gibbon.transform.position;
        simple_pos[1] = 0f;
        simple_pos[2] = 0f;

        // Init hand positions
        limb_targets = new float3[4];
        for(int i=0; i<4; ++i){
            limb_targets[i] = simple_pos;
            walk.limb_targets[i] = simple_pos;
            swing.limb_targets[i] = simple_pos;
        }

        // Get transforms of each skeleton point
        var root = GameObject.Find("points").transform;
        var neck = root.Find("neck");
        var stomach = root.Find("stomach");
        var pelvis = root.Find("pelvis");
        var groin = root.Find("groin");
        var head = root.Find("head");
        var shoulder = root.Find("shoulder");
        var elbow = root.Find("elbow");
        var grip = root.Find("grip");
        var hip = root.Find("hip");
        var knee = root.Find("knee");
        var foot = root.Find("foot");
        
        // Set up bind poses for each bone
        SetBindPart(bind_parts.head, display_gibbon.transform.Find("DEF-head"));
        SetBindPart(bind_parts.chest, display_gibbon.transform.Find("DEF-chest"));
        SetBindPart(bind_parts.belly, display_gibbon.transform.Find("DEF-belly"));
        SetBindPart(bind_parts.pelvis, display_gibbon.transform.Find("DEF-pelvis"));
        SetBindPart(bind_parts.arm_top_l, display_gibbon.transform.Find("DEF-upper_arm_L"));
        SetBindPart(bind_parts.arm_bottom_l, display_gibbon.transform.Find("DEF-forearm_L"));
        SetBindPart(bind_parts.arm_top_r, display_gibbon.transform.Find("DEF-upper_arm_R"));
        SetBindPart(bind_parts.arm_bottom_r, display_gibbon.transform.Find("DEF-forearm_R"));
        SetBindPart(bind_parts.leg_top_l, display_gibbon.transform.Find("DEF-thigh_L"));
        SetBindPart(bind_parts.leg_bottom_l, display_gibbon.transform.Find("DEF-shin_L"));
        SetBindPart(bind_parts.leg_top_r, display_gibbon.transform.Find("DEF-thigh_R"));
        SetBindPart(bind_parts.leg_bottom_r, display_gibbon.transform.Find("DEF-shin_R"));

        // Adjust elbow to match arm transform
        elbow.position = bind_parts.arm_bottom_r.transform.position;

        // Set up initial IK poses (just used to get bone lengths later)
        arm_ik.points[0] = shoulder.position;
        arm_ik.points[1] = elbow.position;
        arm_ik.points[2] = grip.position;
        
        leg_ik.points[0] = hip.position;
        leg_ik.points[1] = bind_parts.leg_bottom_r.transform.position;
        leg_ik.points[2] = foot.position;

        float measured_arm_length = Vector3.Distance(shoulder.position, elbow.position) + Vector3.Distance(elbow.position, grip.position);
            
        // Set up particles and bones for swinging sim
        for(int i=0; i<4; ++i){
            Verlet.System temp;
            switch(i){
                case 0:  temp = arms; break;
                case 1:  temp = walk.arms; break;
                case 2:  temp = jump.arms; break;
                default:  temp = swing.arms; break;
            }
            temp.AddPoint(shoulder.position, "shoulder_r");
            temp.AddPoint(grip.position, "hand_r");
            temp.AddPoint((shoulder.position+Vector3.right * (neck.position[0] - shoulder.position[0])*2f), "shoulder_l");
            temp.AddPoint((grip.position+Vector3.right * (neck.position[0] - grip.position[0])*2f), "hand_l");
            temp.AddPoint(new float3(neck.position[0], hip.position[1], neck.position[2]), "body");
            temp.points[0].mass = 2f;
            temp.points[2].mass = 2f;
            temp.points[4].mass = 4f;
        
            temp.AddBone("arm_r", 0, 1);
            temp.bones[temp.bones.Count-1].length[1] = measured_arm_length;
            temp.bones[temp.bones.Count-1].length[0] *= 0.4f; // Allow arm to flex
            temp.AddBone("arm_l", 2, 3);
            temp.bones[temp.bones.Count-1].length[1] = measured_arm_length;
            temp.bones[temp.bones.Count-1].length[0] *= 0.4f;
            temp.AddBone("tri_top", 0, 2);
            temp.AddBone("tri_r", 0, 4);
            temp.AddBone("tri_l", 2, 4);
        }
        
        // Set up particles and bones for full-body IK
        complete.AddPoint(shoulder.position, "shoulder_r");
        complete.AddPoint(grip.position, "hand_r");
        complete.AddPoint((shoulder.position+Vector3.right * (neck.position[0] - shoulder.position[0])*2f), "shoulder_l");
        complete.AddPoint((grip.position+Vector3.right * (neck.position[0] - grip.position[0])*2f), "hand_l");
        complete.AddPoint(new float3(neck.position[0], hip.position[1], neck.position[2]), "body");
        complete.AddPoint(head.position, "head");
        complete.AddPoint(neck.position, "neck");
        complete.AddPoint(stomach.position, "stomach");
        complete.AddPoint(pelvis.position, "hip");
        complete.AddPoint(groin.position, "groin");
        complete.AddPoint(hip.position, "hip_r");
        complete.AddPoint(foot.position, "foot_r");
        complete.AddPoint(hip.position+Vector3.right * (neck.position[0] - hip.position[0])*2f, "hip_l");
        complete.AddPoint(foot.position+Vector3.right * (neck.position[0] - foot.position[0])*2f, "foot_l");
        
        complete.AddBone("arm_r", 0, 1);
        complete.bones[complete.bones.Count-1].length[1] = measured_arm_length;
        complete.bones[complete.bones.Count-1].length[0] *= 0.4f; // Allow arm to flex
        complete.AddBone("arm_l", 2, 3);
        complete.bones[complete.bones.Count-1].length[1] = measured_arm_length;
        complete.bones[complete.bones.Count-1].length[0] *= 0.4f;
        complete.AddBone("head", 5, 6);
        complete.AddBone("chest", 6, 7);
        complete.AddBone("belly", 7, 8);
        complete.AddBone("pelvis", 8, 9);
        complete.AddBone("leg_r", 10, 11);
        complete.bones[complete.bones.Count-1].length[0] *= 0.4f;
        complete.AddBone("leg_l", 12, 13);
        complete.bones[complete.bones.Count-1].length[0] *= 0.4f;
        
        int num_segments = 40;
        float x = 0;
        float y = 0;
        for(int i=0; i<num_segments+1; ++i){
            branches.AddPoint(new float3(x,y,0), "branch");
            x += UnityEngine.Random.Range(2.0f, 6.0f);
            y += UnityEngine.Random.Range(-3.0f, 3.0f);
            y = math.clamp(y, -2.5f, 2.5f);
        }
        for(int i=0; i<num_segments; ++i){
            branches.AddBone("branch", i, i+1);
        }
    }
        
    // Use law of cosines to find angles of triangle
    static float GetAngleGivenSides(float a, float b, float c){
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

    // Solve two bone IK problems
    static void ApplyTwoBoneIK(int start, int end, float3 forward, TwoBoneIK ik, BindPart top, BindPart bottom, List<Verlet.Point> points, float3 old_axis, float3 axis){
            var shoulder_offset = (points[start].pos - points[start].bind_pos);
            var shoulder_rotation = Quaternion.LookRotation(points[end].pos - points[start].pos, forward) * Quaternion.Inverse(Quaternion.LookRotation(points[end].bind_pos - points[start].bind_pos, Vector3.forward));
        
            float dist_a = math.distance(ik.points[0], ik.points[1]);
            float dist_b = math.distance(ik.points[1], ik.points[2]);
            float dist_c = math.distance(points[start].pos, points[end].pos);
            float old_dist_c = math.distance(ik.points[0], ik.points[2]);
            var old_elbow_angle = GetAngleGivenSides(dist_a, dist_b, old_dist_c);
            var old_shoulder_angle = GetAngleGivenSides(old_dist_c, dist_a, dist_b);
            var elbow_angle = GetAngleGivenSides(dist_a, dist_b, dist_c);
            var shoulder_angle = GetAngleGivenSides(dist_c, dist_a, dist_b);
            DebugText.AddVar("elbow_angle", elbow_angle, 0.5f);
            DebugText.AddVar("shoulder_angle", shoulder_angle, 0.5f);

            // Elbow axis is perpendicular to arm direction and vector from middle of arm to base of neck
            shoulder_rotation = Quaternion.AngleAxis(shoulder_angle * Mathf.Rad2Deg, axis) * shoulder_rotation * 
                                Quaternion.Inverse(Quaternion.AngleAxis(old_shoulder_angle * Mathf.Rad2Deg, old_axis));
            
            top.transform.position = top.bind_pos + shoulder_offset;
            top.transform.rotation = shoulder_rotation * top.bind_rot;
        
            var elbow = top.transform.position + top.transform.rotation * Quaternion.Inverse(top.bind_rot) * (bottom.bind_pos - top.bind_pos);
            bottom.transform.position = elbow;
            bottom.transform.rotation = Quaternion.AngleAxis(elbow_angle * Mathf.Rad2Deg, axis) * shoulder_rotation * 
                                        Quaternion.Inverse(Quaternion.AngleAxis(old_elbow_angle * Mathf.Rad2Deg, old_axis)) * bottom.bind_rot;
        
    }
    
    // Calculate transform based on bone points and character "forward" direction
    void ApplyBound(BindPart part, float3 forward, float3 bind_forward, int start, int end){
        var up = math.normalize(complete.points[end].pos  - complete.points[start].pos);
        var bind_up = math.normalize(complete.points[end].bind_pos  - complete.points[start].bind_pos);       
        var mid = (complete.points[end].pos + complete.points[start].pos)/2.0f;
        var bind_mid = (complete.points[end].bind_pos + complete.points[start].bind_pos)/2.0f;
        
        var rotation = Quaternion.LookRotation(up, forward) * 
                       Quaternion.Inverse(Quaternion.LookRotation(bind_up, bind_forward));
        part.transform.rotation = rotation * part.bind_rot;
        part.transform.position = mid + (float3)(rotation * (part.bind_pos - bind_mid));
    }

    float BranchHeight(float x, int start, int end){
        float branch_t = (x-branches.points[start].bind_pos[0])/(branches.points[end].bind_pos[0]-branches.points[start].bind_pos[0]);
        return math.lerp(branches.points[start].pos[1], branches.points[end].pos[1], branch_t);
    }
    
    float BranchesHeight(float x){
        for(int i=0;i<branches.bones.Count; ++i){
            var bone = branches.bones[i];
            if(x >= branches.points[bone.points[0]].pos[0] && x < branches.points[bone.points[1]].pos[0]){
                return BranchHeight(x, bone.points[0], bone.points[1]);
            }
        }
        if(x < 0.0f){
            return branches.points[0].pos[1];
        } else {
            return branches.points[branches.points.Count-1].pos[1];
        }
    }

    bool in_air;
    float in_air_amount = 0.0f;
    float3 jump_com_offset;
    float jump_time;

    // Prepare to draw next frame
    void Update() {                        
        // Used to test out verlet systems by dragging points around in Scene view
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

        if(Input.GetKeyDown(KeyCode.Space)){
            in_air = true;
            simple_vel[1] = 5.0f;

            float total_mass = 0f;
            var com = float3.zero;
            for(int i=0; i<arms.points.Count; ++i){
                com += arms.points[i].pos * arms.points[i].mass;
                total_mass += arms.points[i].mass;
            }
            com /= total_mass;
            jump_com_offset = com-simple_pos;
            jump_time = Time.time;
        }

        // Use "arms" rig to drive full body IK rig
        const bool map_complete_to_arms = true;
        if(map_complete_to_arms){
            var bind_mid = (arms.points[0].bind_pos + arms.points[2].bind_pos + arms.points[4].bind_pos)/3.0f;
            var mid = (arms.points[0].pos + arms.points[2].pos + arms.points[4].pos)/3.0f;
            var forward = math.normalize(math.cross(arms.points[0].pos - arms.points[2].pos, arms.points[0].pos - arms.points[4].pos));
            var bind_forward = math.normalize(math.cross(arms.points[0].bind_pos - arms.points[2].bind_pos, arms.points[0].bind_pos - arms.points[4].bind_pos));
            var up = math.normalize((arms.points[0].pos + arms.points[2].pos)/2.0f - arms.points[4].pos);
            var bind_up = math.normalize((arms.points[0].bind_pos + arms.points[2].bind_pos)/2.0f - arms.points[4].bind_pos);
        
            complete.points[0].pos = arms.points[0].pos;
            complete.points[1].pos = arms.points[1].pos;
            complete.points[2].pos = arms.points[2].pos;
            complete.points[3].pos = arms.points[3].pos;
            complete.points[0].pinned = true;
            complete.points[1].pinned = true;
            complete.points[2].pinned = true;
            complete.points[3].pinned = true;

            var chest_rotation = math.mul(quaternion.LookRotation(forward, up), 
                                          math.inverse(quaternion.LookRotation(bind_forward, bind_up)));

            for(int i=5; i<14; ++i){
                complete.points[i].pos = mid + math.mul(chest_rotation, (complete.points[i].bind_pos - bind_mid));
                complete.points[i].pinned = true;
            }

            
            for(int i=0; i<2; ++i){
                complete.points[11+i*2].pos = limb_targets[2+i];
            }
            
            complete.Constraints();
        }

        // Apply full body IK rig to visual deformation bones
        {
            // Get torso orientation and position
            var bind_mid = (complete.points[0].bind_pos + complete.points[2].bind_pos + complete.points[9].bind_pos)/3.0f;
            var mid = (complete.points[0].pos + complete.points[2].pos + complete.points[9].pos)/3.0f;
            var forward = -math.normalize(math.cross(complete.points[0].pos - complete.points[2].pos, complete.points[0].pos - complete.points[9].pos));
            var bind_forward = -math.normalize(math.cross(complete.points[0].bind_pos - complete.points[2].bind_pos, complete.points[0].bind_pos - complete.points[9].bind_pos));
            var up = math.normalize((complete.points[0].pos + complete.points[2].pos)/2.0f - complete.points[9].pos);
            var bind_up = math.normalize((complete.points[0].bind_pos + complete.points[2].bind_pos)/2.0f - complete.points[9].bind_pos);
        
            // Apply core bones
            ApplyBound(bind_parts.head, forward, bind_forward, 5, 6);
            ApplyBound(bind_parts.chest, forward, bind_forward, 6, 7);
            ApplyBound(bind_parts.belly, forward, bind_forward, 7, 8);
            ApplyBound(bind_parts.pelvis, forward, bind_forward, 8, 9);

            // Arm IK
            for(int i=0; i<2; ++i){
                var top = bind_parts.arm_top_r;
                var bottom = bind_parts.arm_bottom_r;
                if(i==1){
                    top = bind_parts.arm_top_l;
                    bottom = bind_parts.arm_bottom_l;
                }

                var points = complete.points;
                int start = i*2;
                int end = i*2+1;
                var old_axis = math.normalize(math.cross((points[end].bind_pos+points[start].bind_pos)*0.5f - (points[2].bind_pos + points[0].bind_pos) * 0.5f, points[start].bind_pos - points[end].bind_pos));//shoulder_rotation * Vector3.forward;// math.normalize(shoulder_rotation * math.cross(left_arm.points[2] - left_arm.points[1], left_arm.points[1] - left_arm.points[0]));
                var axis = math.normalize(math.cross((points[end].pos+points[start].pos)*0.5f - (points[2].pos + points[0].pos) * 0.5f, points[start].pos - points[end].pos));//shoulder_rotation * Vector3.forward;// math.normalize(shoulder_rotation * math.cross(left_arm.points[2] - left_arm.points[1], left_arm.points[1] - left_arm.points[0]));
            
                ApplyTwoBoneIK(start, end, forward, arm_ik, top, bottom, complete.points, old_axis, axis);
            }

            // Leg IK
            for(int i=0; i<2; ++i){
                var top = bind_parts.leg_top_r;
                var bottom = bind_parts.leg_bottom_r;
                if(i==1){
                    top = bind_parts.leg_top_l;
                    bottom = bind_parts.leg_bottom_l;
                }
            
                var points = complete.points;
                int start = i*2+10;
                int end = i*2+1+10;

                var bind_shoulder_rotation = Quaternion.LookRotation(points[end].bind_pos - points[start].bind_pos, Vector3.forward);
                var shoulder_rotation = Quaternion.LookRotation(points[end].pos - points[start].pos, forward) * bind_shoulder_rotation;
        
                var old_axis = bind_shoulder_rotation * Vector3.right;
                var axis = shoulder_rotation * Vector3.right;

                ApplyTwoBoneIK(start, end, forward, leg_ik, top, bottom, complete.points, old_axis, axis);
            }
        }

        branches.DrawBones(new Color(0.5f, 0.5f, 0.1f, 1.0f));
        //arms.DrawBones(Color.white);

        if(ImGui.Begin("Gibbon")){
            ImGui.Text($"horz speed: {math.abs(simple_vel[0])}");  
            ImGui.Text($"swing time: {swing_time}");  
            ImGui.SliderFloat("climb_amount", ref climb_amount, 0f, 1f);
            ImGui.SliderFloat("com_offset_amount", ref com_offset_amount, 0f, 1f);
            ImGui.SliderFloat("base_walk_height", ref base_walk_height, 0f, 1f);
            ImGui.SliderFloat("tilt_offset", ref tilt_offset, 0f, 1f);
            ImGui.SliderFloat("arms_up", ref arms_up, 0f, 1f);
        }
        ImGui.End();

        if(Input.GetKeyDown(KeyCode.Tab)){
            Time.timeScale = (Time.timeScale == 1.0f)?0.1f:1.0f;
        }
    }
    
    float3 MoveTowards(float3 a, float3 b, float max_dist){
        float len = math.distance(a,b);
        if(len < max_dist){
            return b;
        } else {
            return a + (b-a)/len*max_dist;
        }
    }

    float climb_amount = 0f;
    float com_offset_amount = 0.25f;
    float base_walk_height = 0.7f;
    float tilt_offset = 0.81f;
    float arms_up = 0.0f;
    bool wants_to_swing = true;

    void Swap(ref float3 a, ref float3 b){
        var temp = a;
        a = b;
        b = temp;
    }
    
    float3 old_test_pos;

    // Apply actual controls and physics
    void Step(float step) {
        // Transform controls to axes
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

        if(vert_input < 0f){
            wants_to_swing = true;
        } else if(vert_input > 0f){
            wants_to_swing = false;
        }

        climb_amount = Mathf.MoveTowards(climb_amount, wants_to_swing?0.0f:1.0f, step * 2f);

        float max_speed = 10f;
        if(!in_air && climb_amount > 0.5f){
            max_speed = 7f;
        }

        // Simple velocity control
        var old_pos = simple_pos;
        simple_vel[0] += horz_input * step * 5f;
        simple_vel[0] = math.clamp(simple_vel[0], -max_speed, max_speed);
        simple_pos += simple_vel * step;
        if(in_air){
            jump_com_offset *= 0.99f;
            simple_vel += (float3)Physics.gravity * step;

            if(simple_vel[1] <= 0.0f && simple_pos[1] < BranchesHeight(simple_pos[0])){
                in_air = false;
            }
        }
        if(!in_air){
            simple_pos[1] = BranchesHeight(simple_pos[0]);
            simple_vel[1] = 0.0f;
        }
        in_air_amount = Mathf.MoveTowards(in_air_amount, in_air?1.0f:0.0f, 1f);// step * 10f);

        var test_pos = simple_pos + simple_vel * 0.1f;
        test_pos[1] = BranchesHeight(test_pos[0]);
        var test_pos2 = simple_pos + simple_vel * -0.1f;
        test_pos2[1] = BranchesHeight(test_pos2[0]);
        var new_pos = (test_pos + test_pos2 + simple_pos) / 3.0f;
        /*DebugDraw.Sphere(simple_pos, Color.yellow, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray);
        DebugDraw.Sphere(test_pos, Color.blue, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray);
        DebugDraw.Sphere(test_pos2, Color.blue, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray);
        DebugDraw.Sphere((test_pos+test_pos2)*0.5f, Color.green, Vector3.one * 0.2f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray);
        DebugDraw.Line(old_test_pos, new_pos, Color.green, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);*/
        old_test_pos = new_pos;
        
        { // swing
            // Adjust amplitude and time scale based on speed
            float amplitude = math.pow(math.abs(simple_vel[0])/10f + 1f, 0.8f)-1f+0.1f;
            float min_height = -1f + amplitude * 0.25f + math.max(0.0f, 0.1f - math.abs(simple_vel[0]) * 0.1f);
            var old_swing_time = swing_time;
            float swing_speed_mult = 8f/(math.PI*2f);
            swing_time += step*swing_speed_mult;
            if(math.ceil(old_swing_time) != math.ceil(swing_time)){
                locked_handhold = limb_targets[next_hand];
                next_hand = 1-next_hand;
            }
                        
            // Figure out target hand positions
            float next_trough_time = ((math.ceil(swing_time)-0.25f))/swing_speed_mult;
            swing.limb_targets[next_hand] = simple_pos + simple_vel * (next_trough_time-Time.time);
            for(int i=0; i<2; ++i){
                swing.limb_targets[i][1] = BranchesHeight(swing.limb_targets[i][0]);
            }
            
            var pendulum_length = 0.9f;
            swing.target_com = simple_pos - simple_vel * 0.05f;
            swing.target_com[0] += (math.cos((swing_time-0.1f) * (math.PI*2f)))* pendulum_length * 0.5f * math.clamp(simple_vel[0] * 0.5f, -1f, 1f) * math.max(0f, 1f - math.abs(simple_vel[0])*2f);
            
            // Smooth out motion so COM follows line between handholds instead of branch itself
            // e.g. movement should be horizontal when swinging from one tip of a V shape to the next
            {
                var points = new float3[3];
                points[0] = swing.limb_targets[0];
                points[1] = swing.limb_targets[1];
                points[2] = swing.limb_targets[next_hand] + simple_vel / swing_speed_mult;
                points[2][1] = BranchesHeight(points[2][0]);
                if(points[0][0] > points[1][0]){
                    Swap(ref points[0], ref points[1]);
                }
                if(points[1][0] > points[2][0]){
                    Swap(ref points[1], ref points[2]);
                }
                if(points[0][0] > points[1][0]){
                    Swap(ref points[0], ref points[1]);
                }

                //DebugDraw.Line(points[0], points[1], Color.cyan, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray );
                //DebugDraw.Line(points[1], points[2], Color.gray, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray );
        
                if(swing.target_com[0] < points[1][0]){
                    float interp = math.max(0, (swing.target_com[0]-points[0][0]) / (points[1][0] - points[0][0]));
                    swing.target_com[1] = math.lerp(points[0][1], points[1][1], interp);
                } else {
                    float interp = math.min(1,(swing.target_com[0]-points[1][0]) / (points[2][0] - points[1][0]));
                    swing.target_com[1] = math.lerp(points[1][1], points[2][1], interp);
                }
            }

            swing.target_com[1] += (min_height + (math.sin((swing_time-0.1f) * (math.PI*2f))+1f)*amplitude) * pendulum_length;
            
            float pull_up = climb_amount;
            swing.target_com[1] += pull_up;
            

            //DebugDraw.Sphere(swing.limb_targets[0], Color.green, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray );
            //DebugDraw.Sphere(swing.limb_targets[1], Color.blue, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray );
            
            if(in_air){
                swing.arms.StartSim(step);
                for(int i=0; i<arms.points.Count; ++i){
                    swing.arms.points[i].pos = arms.points[i].pos;
                }
                swing.arms.EndSim();
            } else {
                var arms = swing.arms;
                // Use COM and hand positions to drive arm rig
                // Move hands towards grip targets
                for(int i=0; i<2; ++i){
                    arms.points[i*2+1].pos = MoveTowards(arms.points[i*2+1].pos, swing.limb_targets[i], math.max(0f, math.cos((swing_time+0.35f+(1-i))*math.PI*1f)*0.5f+0.5f) * step * 5f);
                    arms.points[i*2+1].old_pos = math.lerp(arms.points[i*2+1].old_pos, arms.points[i*2+1].pos - simple_vel*step, 0.25f);
                }
                arms.StartSim(step);
                for(int j=0; j<4; ++j){
                    // Adjust all free points to match target COM
                    float total_mass = 0f;
                    var com = float3.zero;
                    for(int i=0; i<arms.points.Count; ++i){
                        if(arms.points[i].pinned == false){
                            com += arms.points[i].pos * arms.points[i].mass;
                            total_mass += arms.points[i].mass;
                        }
                    }
                    com /= total_mass;
                    var offset = swing.target_com - com;
                    for(int i=0; i<arms.points.Count; ++i){
                        if(i!=1 && i!=3){
                            arms.points[i].pos += offset;
                        }
                    }
                    // Apply torque to keep torso upright and forward-facing
                    float step_sqrd = step*step;
                    float force = 20f;
                    arms.points[4].pos[1] -= step_sqrd * force;
                    arms.points[0].pos[1] += step_sqrd * force * 0.5f;
                    arms.points[2].pos[1] += step_sqrd * force * 0.5f;
                    arms.points[0].pos[2] -= step_sqrd * simple_vel[0] * 2.0f;
                    arms.points[2].pos[2] += step_sqrd * simple_vel[0] * 2.0f;
                    arms.points[4].pos[0] -= simple_vel[0] * step_sqrd * 2f; // Apply backwards force to maintain forwards tilt
                    arms.Constraints();
                }
                arms.EndSim();
                
                var up = math.normalize((arms.points[0].pos+arms.points[2].pos)*0.5f-arms.points[4].pos);
                for(int i=0; i<2; ++i){
                    swing.limb_targets[2+i] = arms.points[i*2].pos - up + up * (0.35f + 0.15f * math.sin((swing_time + i*1.0f)*math.PI));
                }
            }
        }
        
        { // jump
            jump.target_com = simple_pos + jump_com_offset;

            if(!in_air){
                jump.arms.StartSim(step);
                for(int i=0; i<arms.points.Count; ++i){
                    jump.arms.points[i].pos = arms.points[i].pos;
                }
                jump.arms.EndSim();
            } else {
            var arms = jump.arms;
            // Use COM and hand positions to drive arm rig
            bool arms_map = true;
            if(arms_map){
                for(int i=0; i<2; ++i){
                    arms.points[i*2+1].old_pos = math.lerp(arms.points[i*2+1].old_pos, arms.points[i*2+1].pos - simple_vel*step, 0.75f);
                }
                // Move hands towards grip targets
                arms.StartSim(step);
                for(int j=0; j<4; ++j){
                    // Adjust all free points to match target COM
                    float total_mass = 0f;
                    var com = float3.zero;
                    for(int i=0; i<arms.points.Count; ++i){
                        com += arms.points[i].pos * arms.points[i].mass;
                        total_mass += arms.points[i].mass;
                    }
                    com /= total_mass;
                    var offset = jump.target_com - com;
                    for(int i=0; i<arms.points.Count; ++i){
                        arms.points[i].pos += offset;
                    }
                    // Apply torque to keep torso upright and forward-facing
                    float step_sqrd = step*step;
                    float force = 10f;
                    arms.points[4].pos[1] -= step_sqrd * force;
                    arms.points[0].pos[1] += step_sqrd * force * 0.5f;
                    arms.points[2].pos[1] += step_sqrd * force * 0.5f;
                    arms.points[0].pos[2] -= step_sqrd * simple_vel[0] * 1.0f;
                    arms.points[2].pos[2] += step_sqrd * simple_vel[0] * 1.0f;
                    arms.points[4].pos[0] -= simple_vel[0] * step_sqrd * 1f; // Apply backwards force to maintain forwards tilt
                    arms.Constraints();
                }
                arms.EndSim();
                
                var up = math.normalize((arms.points[0].pos+arms.points[2].pos)*0.5f-arms.points[4].pos);
                var forward = math.normalize(math.cross(arms.points[0].pos - arms.points[2].pos, arms.points[0].pos - arms.points[4].pos));
                for(int i=0; i<2; ++i){
                    jump.limb_targets[2+i] = arms.points[i*2].pos - up + (up * 0.5f + forward * 0.1f) * math.min(1.0f, Time.time - jump_time);
                }
            }
            }
        }

        bool calc_walk = true;
        if(calc_walk){
            float speed_mult = 8f/(math.PI*2f) * math.pow((math.abs(simple_vel[0])+1.0f),0.4f);
            walk_time += step*speed_mult;

            var target_com = simple_pos;
            target_com[1] = new_pos[1];
            float crouch_amount = 1.0f-climb_amount;
            target_com[1] += math.lerp(base_walk_height, 0.3f, crouch_amount) + math.sin((walk_time+com_offset_amount) * math.PI * 4.0f) * math.abs(simple_vel[0]) * 0.015f / speed_mult + math.abs(simple_vel[0])*0.01f;
            
            if(in_air){
                walk.arms.StartSim(step);
                for(int i=0; i<arms.points.Count; ++i){
                    walk.arms.points[i].pos = arms.points[i].pos;
                }
                walk.arms.EndSim();
            } else {
                var arms = walk.arms;
                arms.StartSim(step);
                for(int j=0; j<4; ++j){
                    // Adjust all free points to match target COM
                    float total_mass = 0f;
                    var com = float3.zero;
                    for(int i=0; i<arms.points.Count; ++i){
                        if(i!=1 && i!=3){
                            com += arms.points[i].pos * arms.points[i].mass;
                            total_mass += arms.points[i].mass;
                        }
                    }
                    com /= total_mass;
                    var offset = (float3)target_com - com;
                    for(int i=0; i<arms.points.Count; ++i){
                        //if(i!=1 && i!=3){
                            arms.points[i].pos += offset;
                        //}
                    }
                    // Apply torque to keep torso upright and forward-facing
                    float step_sqrd = step*step;
                    float force = 20f;
                    var forward = math.normalize(math.cross(arms.points[0].pos - arms.points[2].pos, arms.points[0].pos - arms.points[4].pos));
                    float3 top_force = new float3(0,1,0) * force;
                    top_force += forward * 5.0f;
                    arms.points[4].pos += step_sqrd * -top_force;
                    arms.points[0].pos += step_sqrd * top_force * 0.5f;
                    arms.points[2].pos += step_sqrd * top_force * 0.5f;
                    arms.points[0].pos[2] -= step_sqrd * simple_vel[0] * 2.0f;
                    arms.points[2].pos[2] += step_sqrd * simple_vel[0] * 2.0f;
                    
                    for(int i=0; i<2; ++i){
                        arms.points[i*2].pos[0] += step_sqrd * -3.0f * (math.cos((walk_time + tilt_offset) * math.PI * 2.0f + math.PI*i))*0.2f * simple_vel[0] / speed_mult;
                    }
                    // Move arms out to sides
                    float speed = math.abs(simple_vel[0])/max_speed;
                    for(int i=0; i<2; ++i){
                        arms_up = math.abs(speed * (math.sin(Time.time * ((i==1)?2.5f:2.3f))*0.25f+0.75f));
                        arms.points[1+i*2].pos += step_sqrd * (arms.points[0].pos - arms.points[2].pos) * (1.5f+speed*2.0f+arms_up*2.0f) * (1-i*2);
                        arms.points[1+i*2].pos -= step_sqrd * forward * 3.0f * arms_up;;
                        arms.points[1+i*2].pos[1] -= step_sqrd * 10.0f * (1.0f - arms_up);
                        arms.bones[i].length[1] = arms.bones[0].length[0] / 0.4f * (0.95f + math.sin(arms_up * math.PI)*0.05f);
                    }

                    arms.Constraints();
                }
                arms.EndSim();
                for(int i=0; i<2; ++i){
                    walk.limb_targets[2+i] = simple_pos;
                    walk.limb_targets[2+i][1] += (-math.sin(walk_time * math.PI * 2.0f + math.PI*i) + 1.0f)*0.2f * (math.pow(math.abs(simple_vel[0]) + 1.0f, 0.3f) - 1.0f);
                    var left = simple_pos - new float3(0.1f,0.0f,0.0f);
                    var right = simple_pos + new float3(0.1f,0.0f,0.0f);
                    left[1] = BranchesHeight(left[0]);
                    right[1] = BranchesHeight(right[0]);
                    float3 move_dir = math.normalize(right - left);
                    walk.limb_targets[2+i] += (move_dir * (math.cos(walk_time * math.PI * 2.0f + math.PI*i))*0.2f - 0.03f) * simple_vel[0] / speed_mult;
                    walk.limb_targets[2+i] += (arms.points[0].pos - arms.points[2].pos) * (1.0f-2.0f*i) * 0.3f;
                }
            }
        }
        
        {
            var old_com = float3.zero;
            {
                float total_mass = 0.0f;
                for(int i=0; i<arms.points.Count; ++i){
                    if(arms.points[i].pinned == false){
                        old_com += arms.points[i].pos * arms.points[i].mass;
                        total_mass += arms.points[i].mass;
                    }
                }
                old_com /= total_mass;
            }

            for(int i=0; i<arms.points.Count; ++i){
                arms.points[i].pos = math.lerp(swing.arms.points[i].pos, walk.arms.points[i].pos, climb_amount);
                arms.points[i].pos = math.lerp(arms.points[i].pos, jump.arms.points[i].pos, in_air_amount);
            }
            arms.Constraints();
            for(int i=0; i<4; ++i){
                limb_targets[i] = math.lerp(swing.limb_targets[i], walk.limb_targets[i], climb_amount);
                limb_targets[i] = math.lerp(limb_targets[i], jump.limb_targets[i], in_air_amount);
            }

            var com = float3.zero;
            {
                float total_mass = 0.0f;
                for(int i=0; i<arms.points.Count; ++i){
                    if(arms.points[i].pinned == false){
                        com += arms.points[i].pos * arms.points[i].mass;
                        total_mass += arms.points[i].mass;
                    }
                }
                com /= total_mass;
            }
            //DebugDraw.Line(old_com, com, Color.green, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
        }

        // Move game camera to track character
        var cam_pos = Camera.main.transform.position;
        cam_pos[0] = simple_pos[0];
        Camera.main.transform.position = cam_pos;
    }

    private void FixedUpdate() {
        Step(Time.fixedDeltaTime);
    }
}
}