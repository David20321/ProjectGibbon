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
        
    // Skinning data
    class DisplayBone {
        public Transform transform;
        public quaternion bind_rot;
        public float3 bind_pos;

        public void Bind(Transform transform){
            this.transform = transform;
            bind_pos = transform.position;
            bind_rot = transform.rotation;
        }
    }

    class DisplayBody {
        public DisplayBone chest = new DisplayBone();
        public DisplayBone arm_top_l = new DisplayBone();
        public DisplayBone arm_bottom_l = new DisplayBone();
        public DisplayBone arm_top_r = new DisplayBone();
        public DisplayBone arm_bottom_r = new DisplayBone();
        public DisplayBone head = new DisplayBone();
        public DisplayBone belly = new DisplayBone();
        public DisplayBone pelvis = new DisplayBone();
        public DisplayBone leg_top_l = new DisplayBone();
        public DisplayBone leg_bottom_l = new DisplayBone();
        public DisplayBone leg_top_r = new DisplayBone();
        public DisplayBone leg_bottom_r = new DisplayBone();
    }

    DisplayBody display_body = new DisplayBody();
    
    // Particle simulation systems
    Verlet.System complete = new Verlet.System();
    Verlet.System branches = new Verlet.System();

    // Initial point positions for IK
    float3[] arm_ik = new float3[3];
    float3[] leg_ik = new float3[3];

    // Simple character particle information
    float3 simple_pos;
    float3 simple_vel = float3.zero;

    // Position in animation sequences
    float swing_time = 0f;
    float walk_time = 0f;

    class MovementSystem {
        public float3 target_com;
        public float3[] limb_targets = new float3[4];
        public Verlet.System arms = new Verlet.System();
        public float body_compress_amount = 0.0f;
    }

    MovementSystem walk = new MovementSystem();
    MovementSystem swing = new MovementSystem();
    MovementSystem jump = new MovementSystem();
    MovementSystem display = new MovementSystem();

    void Start() {
        // Starting point
        simple_pos = display_gibbon.transform.position;
        simple_pos[1] = 0f;
        simple_pos[2] = 0f;

        // Init hand positions
        for(int i=0; i<4; ++i){
            display.limb_targets[i] = simple_pos;
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
        display_body.head.Bind(display_gibbon.transform.Find("DEF-head"));
        display_body.chest.Bind(display_gibbon.transform.Find("DEF-chest"));
        display_body.belly.Bind(display_gibbon.transform.Find("DEF-belly"));
        display_body.pelvis.Bind(display_gibbon.transform.Find("DEF-pelvis"));
        display_body.arm_top_l.Bind(display_gibbon.transform.Find("DEF-upper_arm_L"));
        display_body.arm_bottom_l.Bind(display_gibbon.transform.Find("DEF-forearm_L"));
        display_body.arm_top_r.Bind(display_gibbon.transform.Find("DEF-upper_arm_R"));
        display_body.arm_bottom_r.Bind(display_gibbon.transform.Find("DEF-forearm_R"));
        display_body.leg_top_l.Bind(display_gibbon.transform.Find("DEF-thigh_L"));
        display_body.leg_bottom_l.Bind(display_gibbon.transform.Find("DEF-shin_L"));
        display_body.leg_top_r.Bind(display_gibbon.transform.Find("DEF-thigh_R"));
        display_body.leg_bottom_r.Bind(display_gibbon.transform.Find("DEF-shin_R"));

        // Adjust elbow to match arm transform
        elbow.position = display_body.arm_bottom_r.transform.position;

        // Set up initial IK poses (just used to get bone lengths later)
        arm_ik[0] = shoulder.position;
        arm_ik[1] = elbow.position;
        arm_ik[2] = grip.position;
        
        leg_ik[0] = hip.position;
        leg_ik[1] = display_body.leg_bottom_r.transform.position;
        leg_ik[2] = foot.position;

        float measured_arm_length = Vector3.Distance(shoulder.position, elbow.position) + Vector3.Distance(elbow.position, grip.position);
            
        // Point ids
        const int p_shoulder_r = 0;
        const int p_hand_r =     1;
        const int p_shoulder_l = 2;
        const int p_hand_l =     3;
        const int p_base =       4;

        // Set up movement system particles and bones
        for(int i=0; i<4; ++i){
            Verlet.System temp;
            switch(i){
                case 0:  temp = display.arms; break;
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
        
        // Set up full-body IK particles and bones
        complete.AddPoint(shoulder.position, "shoulder_r");
        complete.AddPoint(grip.position, "hand_r");
        complete.AddPoint((shoulder.position+Vector3.right * (neck.position[0] - shoulder.position[0])*2f), "shoulder_l");
        complete.AddPoint((grip.position+Vector3.right * (neck.position[0] - grip.position[0])*2f), "hand_l");
        complete.AddPoint(new float3(neck.position[0], hip.position[1], neck.position[2]), "body");
        complete.AddPoint(head.position, "head");
        complete.AddPoint(neck.position, "neck");
        complete.AddPoint(stomach.position, "stomach"); // 7
        complete.AddPoint(pelvis.position, "hip"); // 8
        complete.AddPoint(groin.position, "groin");
        complete.AddPoint(hip.position, "hip_r");
        complete.AddPoint(foot.position, "foot_r");
        complete.AddPoint(hip.position+Vector3.right * (neck.position[0] - hip.position[0])*2f, "hip_l");
        complete.AddPoint(foot.position+Vector3.right * (neck.position[0] - foot.position[0])*2f, "foot_l");
        
        complete.AddBone("arm_r", 0, 1);
        complete.bones[complete.bones.Count-1].length[1] = measured_arm_length;
        complete.bones[complete.bones.Count-1].length[0] *= 0.4f;
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
        
        // Create random branch 'terrain'
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
        
        // Delete visible points so we don't see it when playing game
        Destroy(root.gameObject);
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
        if(divisor == 0f){
            return 0f;
        }
        return math.acos(math.clamp(top / divisor, -1f, 1f));
    }

    // Solve two bone IK problems
    static void ApplyTwoBoneIK(int start_id, 
                               int end_id, 
                               float3 forward, 
                               float3[] ik, 
                               DisplayBone top, 
                               DisplayBone bottom, 
                               List<Verlet.Point> points, 
                               float3 old_axis, 
                               float3 axis)
    {
        var start = points[start_id];
        var end = points[end_id];
            
        // Get sides of triangle formed by upper and lower limb
        float dist_a =     math.distance(ik[0], ik[1]);
        float dist_b =     math.distance(ik[1], ik[2]);
        float dist_c =     math.distance(start.pos, end.pos);
        float old_dist_c = math.distance(ik[0], ik[2]);

        // Get angles of triangle
        var old_hinge_angle = GetAngleGivenSides(dist_a,     dist_b, old_dist_c);
        var hinge_angle     = GetAngleGivenSides(dist_a,     dist_b, dist_c);
        var old_base_angle  = GetAngleGivenSides(old_dist_c, dist_a, dist_b);
        var base_angle      = GetAngleGivenSides(dist_c,     dist_a, dist_b);

        // Elbow axis is perpendicular to arm direction and vector from middle of arm to base of neck
        var base_rotation = Quaternion.LookRotation(end.pos - start.pos, forward) * Quaternion.Inverse(Quaternion.LookRotation(end.bind_pos - start.bind_pos, Vector3.forward));
        base_rotation = Quaternion.AngleAxis(base_angle * Mathf.Rad2Deg, axis) * base_rotation * 
                            Quaternion.Inverse(Quaternion.AngleAxis(old_base_angle * Mathf.Rad2Deg, old_axis));
            
        top.transform.position = top.bind_pos + (start.pos - start.bind_pos);
        top.transform.rotation = base_rotation * top.bind_rot;
        
        var elbow = top.transform.position + top.transform.rotation * Quaternion.Inverse(top.bind_rot) * (bottom.bind_pos - top.bind_pos);
        bottom.transform.position = elbow;
        bottom.transform.rotation = Quaternion.AngleAxis(hinge_angle * Mathf.Rad2Deg, axis) * base_rotation * 
                                    Quaternion.Inverse(Quaternion.AngleAxis(old_hinge_angle * Mathf.Rad2Deg, old_axis)) * bottom.bind_rot;        
    }
    
    // Calculate transform based on bone points and character "forward" direction
    void ApplyBound(DisplayBone part, float3 forward, float3 bind_forward, int start, int end){
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
    float predicted_land_time;
    float3 predicted_land_point;
    float3 jump_point;
    float3 look_target;
    float lean = 0.0f;
    
    float forward_amount = 0.1f;
    float up_amount = 0.1f;

    // Prepare to draw next frame
    void Update() {                        
        // Used to test out verlet systems by dragging points around in Scene view
        const bool manually_drag_points = false;
        if(manually_drag_points){
            for(int i=0; i<complete.points.Count; ++i){
                complete.points[i].pos = complete.points[i].widget.position;
            }
            complete.Constraints();
            for(int i=0; i<complete.points.Count; ++i){
                complete.points[i].widget.position = complete.points[i].pos;
            }
        }

        if(Input.GetKeyDown(KeyCode.Space)){
            simple_vel[1] = 5.0f;
            if(!in_air && climb_amount == 0.0f){
                simple_vel[1] += 2.0f;                
            }
            in_air = true;

            float total_mass = 0f;
            var com = float3.zero;
            for(int i=0; i<display.arms.points.Count; ++i){
                com += display.arms.points[i].pos * display.arms.points[i].mass;
                total_mass += display.arms.points[i].mass;
            }
            com /= total_mass;
            jump_com_offset = com-simple_pos;
            jump_time = Time.time;
            jump_point = (display.limb_targets[2]+display.limb_targets[3])*0.5f;
            predicted_land_time = jump_time + 5.0f;
        }

        // Use "arms" rig to drive full body IK rig
        const bool map_complete_to_arms = true;
        if(map_complete_to_arms){
            var arms = display.arms;
            var points = arms.points;

            var bind_mid = (points[0].bind_pos + points[2].bind_pos + points[4].bind_pos)/3.0f;
            var mid = (points[0].pos + points[2].pos + points[4].pos)/3.0f;
            var forward = math.normalize(math.cross(points[0].pos - points[2].pos, points[0].pos - points[4].pos));
            var bind_forward = math.normalize(math.cross(points[0].bind_pos - points[2].bind_pos, points[0].bind_pos - points[4].bind_pos));
            var up = math.normalize((points[0].pos + points[2].pos)/2.0f - points[4].pos);
            var bind_up = math.normalize((points[0].bind_pos + points[2].bind_pos)/2.0f - points[4].bind_pos);
        
            complete.points[0].pos = points[0].pos;
            complete.points[1].pos = points[1].pos;
            complete.points[2].pos = points[2].pos;
            complete.points[3].pos = points[3].pos;
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
            
            complete.points[7].pinned = false;
            complete.points[8].pinned = false;
            var old_hip = complete.points[9].pos;
            for(int i=7; i<=9; ++i){
                complete.points[i].pos = math.lerp(complete.points[i].pos, complete.points[6].pos, body_compress_amount);
            }
            complete.points[7].pos -= forward * body_compress_amount * 0.2f;
            complete.points[8].pos -= forward * body_compress_amount * 0.2f;
            
            
            for(int i=10; i<14; ++i){
                complete.points[i].pos += complete.points[9].pos - old_hip;
            }

            for(int i=0; i<2; ++i){
                complete.points[11+i*2].pos = display.limb_targets[2+i];
            }
            
            for(int i=0; i<2; ++i){
                complete.Constraints();
            }
        } else {
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
            ApplyBound(display_body.head, forward, bind_forward, 5, 6);
            ApplyBound(display_body.chest, forward, bind_forward, 6, 7);
            ApplyBound(display_body.belly, forward, bind_forward, 7, 8);
            ApplyBound(display_body.pelvis, forward, bind_forward, 8, 9);

            // Arm IK
            for(int i=0; i<2; ++i){
                var top = display_body.arm_top_r;
                var bottom = display_body.arm_bottom_r;
                if(i==1){
                    top = display_body.arm_top_l;
                    bottom = display_body.arm_bottom_l;
                }

                var points = complete.points;
                int start = i*2;
                int end = i*2+1;
                forward_amount = -climb_amount * 0.8f;
                up_amount = 0.1f + climb_amount * 0.5f;
                var elbow_point = ((points[2].pos + points[0].pos) * 0.5f + up * up_amount + forward * forward_amount);
                //DebugDraw.Sphere(elbow_point, Color.red, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);
                var old_axis = math.normalize(math.cross((points[end].bind_pos+points[start].bind_pos)*0.5f - ((points[2].bind_pos + points[0].bind_pos) * 0.5f + bind_up * up_amount + bind_forward * forward_amount), points[start].bind_pos - points[end].bind_pos));//shoulder_rotation * Vector3.forward;// math.normalize(shoulder_rotation * math.cross(left_arm.points[2] - left_arm.points[1], left_arm.points[1] - left_arm.points[0]));
                var axis = math.normalize(math.cross((points[end].pos+points[start].pos)*0.5f - elbow_point, points[start].pos - points[end].pos));//shoulder_rotation * Vector3.forward;// math.normalize(shoulder_rotation * math.cross(left_arm.points[2] - left_arm.points[1], left_arm.points[1] - left_arm.points[0]));
            
                ApplyTwoBoneIK(start, end, forward, arm_ik, top, bottom, complete.points, old_axis, axis);
            }

            // Leg IK
            for(int i=0; i<2; ++i){
                var top = display_body.leg_top_r;
                var bottom = display_body.leg_bottom_r;
                if(i==1){
                    top = display_body.leg_top_l;
                    bottom = display_body.leg_bottom_l;
                }
            
                var points = complete.points;
                int start = i*2+10;
                int end = i*2+1+10;

                var bind_shoulder_rotation = Quaternion.LookRotation(points[end].bind_pos - points[start].bind_pos, Vector3.forward);
                
                var leg_dir = points[end].pos - points[start].pos;
                var leg_dir_flat = math.normalize(new float2(math.dot(leg_dir, forward), math.dot(leg_dir, up)));
                var leg_forward = leg_dir_flat[0] * up + leg_dir_flat[1] * -forward;
                var mid_leg = (points[end].pos + points[start].pos)*0.5f;

                //DebugDraw.Line(mid_leg, mid_leg+leg_forward, Color.red, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);

                var shoulder_rotation = Quaternion.LookRotation(leg_dir, leg_forward) * bind_shoulder_rotation;
        
                var old_axis = bind_shoulder_rotation * Vector3.right;
                var axis = shoulder_rotation * Vector3.right;

                ApplyTwoBoneIK(start, end, leg_forward, leg_ik, top, bottom, complete.points, old_axis, axis);
            }

            // Headlook
            
            // head_look_y: 50 = max look down, -70 = max look up
            // head_look_x: -90 to 90
            var target = math.normalize(display_body.head.transform.InverseTransformPoint(look_target));
            head_look_y = math.sin(target.x)*Mathf.Rad2Deg;
            var temp = target;
            temp.x = 0.0f;
            temp = math.normalize(temp);
            head_look_x = -math.sin(temp.y)*Mathf.Rad2Deg;
            display_body.head.transform.rotation = display_body.head.transform.rotation * Quaternion.Euler(head_look_x, 0f, 0f) * Quaternion.Euler(0f, head_look_y, 0f);
            if(head_look_y > 0.0f){
                display_body.head.transform.position = display_body.head.transform.position + (Vector3)((display_body.head.transform.right) * head_look_y * -0.001f);
            }
        }

        branches.DrawBones(new Color(0.5f, 0.5f, 0.1f, 1.0f));
        //arms.DrawBones(Color.white);
        //complete.DrawBones(Color.white);

        if(false){
            if(ImGui.Begin("Gibbon")){
                ImGui.SliderFloat("lean", ref lean, -1f, 1f);
                ImGui.SliderFloat("forward_amount", ref forward_amount, -1f, 1f);
                ImGui.SliderFloat("up_amount", ref up_amount, -1f, 1f);
                ImGui.SliderFloat("gallop_offset", ref gallop_offset, -1f, 1f);
                ImGui.SliderFloat("gallop_stride", ref gallop_stride, 0f, 0.85f);
                ImGui.SliderFloat("gallop_height_offset", ref gallop_height_offset, 0f, 1f);
                ImGui.SliderFloat("gallop_height", ref gallop_height, 0f, 1f);
                ImGui.SliderFloat("gallop_height_base", ref gallop_height_base, 0f, 1f);
                ImGui.SliderFloat("gallop_hip_rotate", ref gallop_hip_rotate, -4f, 4f);
                ImGui.SliderFloat("gallop_lean", ref gallop_lean, -4f, 4f);
                ImGui.SliderFloat("gallop_stride_height", ref gallop_stride_height, -1f, 1f);
                ImGui.SliderFloat("gallop_arm_stride", ref gallop_arm_stride, 0f, 4f);
                ImGui.SliderFloat("gallop_arm_stride_height", ref gallop_arm_stride_height, 0f, 1f);
                ImGui.SliderFloat("quad_amount", ref quad_amount, 0f, 1f);
                ImGui.SliderFloat("quad_gallop_body_compress_amount", ref quad_gallop_body_compress_amount, 0f, 1f);
                ImGui.SliderFloat("quad_gallop_body_compress_offset", ref quad_gallop_body_compress_offset, 0f, 1f);
                ImGui.SliderFloat("gallop_amount", ref gallop_amount, 0f, 1f);
            }
            ImGui.End();
        }

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

    float climb_amount = 1f;
    float head_look_x = 0.0f;
    float head_look_y = 0.0f;
    float body_compress_amount = 0.0f;
    float base_walk_height = 0.7f;
    float tilt_offset = 0.81f;
    float arms_up = 0.0f;
    bool wants_to_swing = false;
    float skate_amount = 0.0f;
    float gallop_offset = 0.55f; // For biped gallop
    float quad_gallop_offset = 0.25f; // For quadruped gallop
    float gallop_stride = 1.0f;
    float gallop_stride_height = 0.2f;
    float gallop_hip_rotate = -1.3f;
    float gallop_height_offset = 0.6f;
    float gallop_height = 0.012f;
    float gallop_height_base = 0.8f;
    float gallop_lean = 1.5f;
    float gallop_arm_stride_height = 0.4f;
    float gallop_arm_stride = 0.4f;
    float quad_amount = 0.0f;
    float gallop_amount = 0.0f;
    float quad_gallop_body_compress_offset = 0.4f;
    float quad_gallop_body_compress_amount = 0.15f;

    void Swap(ref float3 a, ref float3 b){
        var temp = a;
        a = b;
        b = temp;
    }
    
    float3 old_test_pos;
    float start_time = 0.0f;

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
            if(climb_amount == 1.0f){
                swing_time = start_time;
            }
        }

        if(!in_air){
            if(vert_input < 0f){
                wants_to_swing = true;
            } else if(vert_input > 0f){
                wants_to_swing = false;
            }
        }

        climb_amount = Mathf.MoveTowards(climb_amount, wants_to_swing?0.0f:1.0f, step * 2f);

        float max_speed = 10f - climb_amount*3.0f;

        // Simple velocity control
        var old_pos = simple_pos;
        if(!in_air){
            simple_vel[0] += horz_input * step * 5f;
        }
        simple_vel[0] = math.clamp(simple_vel[0], -max_speed, max_speed);
        
        if(horz_input == 0f && math.abs(simple_vel[0]) < 1.0f){
            simple_vel[0] = Mathf.MoveTowards(simple_vel[0], simple_vel[0]>=0.0f?1.0f:-1.0f, step);
        }

        var test_pos = simple_pos + simple_vel * 0.1f;
        test_pos[1] = BranchesHeight(test_pos[0]);
        var test_pos2 = simple_pos + simple_vel * -0.1f;
        test_pos2[1] = BranchesHeight(test_pos2[0]);
        var new_pos = (test_pos + test_pos2 + simple_pos) / 3.0f;

        var slope_vec = math.normalizesafe(test_pos - simple_pos);
        float slope_speed_mult = math.abs(slope_vec[0]);
        if(climb_amount < 0.5f || in_air){
            slope_speed_mult = 1.0f;
        }

        var effective_vel = simple_vel * slope_speed_mult;

        simple_pos += effective_vel * step;
        if(in_air){
            jump_com_offset *= 0.99f;
            simple_vel += (float3)Physics.gravity * step;

            var traj_vel = simple_vel;
            var traj_pos = simple_pos;
            for(int i=0; i<200; ++i){
                traj_pos += traj_vel * step;
                traj_vel += (float3)Physics.gravity * step;
                //DebugDraw.Sphere(traj_pos, Color.green, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray);
                predicted_land_point = traj_pos;
                if(traj_vel[1] <= 0.0f && traj_pos[1] < BranchesHeight(traj_pos[0])){
                    predicted_land_time = Time.time + step * i;
                    predicted_land_point[1] = BranchesHeight(predicted_land_point[0]);
                    break;
                }
            }

            var test_point = display.limb_targets[2] + simple_vel * step;
            if(simple_vel[1] <= 0.0f && test_point[1] < BranchesHeight(test_point[0])){
                in_air = false;
                skate_amount = 1.0f;
                wants_to_swing = true;
                climb_amount = 1.0f;
                swing_time = 0.0f;
                for(int i=0; i<display.arms.points.Count; ++i){
                    walk.arms.points[i].pos = jump.arms.points[i].pos;
                }
            }
            look_target = predicted_land_point;
        }
        if(!in_air){
            simple_pos[1] = BranchesHeight(simple_pos[0]);
            simple_vel[1] = 0.0f;
            var forward = math.normalize(math.cross(display.arms.points[0].pos - display.arms.points[2].pos, display.arms.points[0].pos - display.arms.points[4].pos));
            look_target[2] += forward[2];
            look_target = (float3)display_body.head.transform.position + forward * 0.1f;
            look_target += test_pos - test_pos2;
        }

        in_air_amount = Mathf.MoveTowards(in_air_amount, in_air?1.0f:0.0f, 1f);// step * 10f);

        //DebugDraw.Sphere(look_target, Color.yellow, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray);
       
        //DebugDraw.Sphere(simple_pos, Color.yellow, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray);
        /*DebugDraw.Sphere(test_pos, Color.blue, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray);
        DebugDraw.Sphere(test_pos2, Color.blue, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray);
        DebugDraw.Sphere((test_pos+test_pos2)*0.5f, Color.green, Vector3.one * 0.2f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray);
        DebugDraw.Line(old_test_pos, new_pos, Color.green, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);*/
        old_test_pos = new_pos;
        
        if(Input.GetKey(KeyCode.J)){
            swing_time = 0.6f;
        }

        { // swing
            // Adjust amplitude and time scale based on speed
            float amplitude = math.pow(math.abs(simple_vel[0])/10f + 1f, 0.8f)-1f+0.1f;
            float min_height = -1f + amplitude * 0.25f + math.max(0.0f, 0.1f - math.abs(simple_vel[0]) * 0.1f);
            var old_swing_time = swing_time;
            float swing_speed_mult = 8f/(math.PI*2f);
            swing_time += step*swing_speed_mult;
            var next_hand = ((int)swing_time)%2;
                        
            // Figure out target hand positions
            float next_trough_time = ((math.ceil(swing_time)-0.25f));
            swing.limb_targets[next_hand] = simple_pos + simple_vel * (next_trough_time-swing_time)/swing_speed_mult;
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
                for(int i=0; i<display.arms.points.Count; ++i){
                    swing.arms.points[i].pos = display.arms.points[i].pos;
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
            
            float dist_from_land = 1.0f - (1.0f / (1.0f + math.max(0.0f, math.min(Time.time - jump_time, predicted_land_time - Time.time)) * 4.0f));

            jump.body_compress_amount = dist_from_land * 0.1f;

            if(!in_air){
                jump.arms.StartSim(step);
                for(int i=0; i<display.arms.points.Count; ++i){
                    jump.arms.points[i].pos = display.arms.points[i].pos;
                }
                jump.arms.EndSim();
            } else {
            var arms = jump.arms;
            // Use COM and hand positions to drive arm rig
            bool arms_map = true;
            if(arms_map){
                for(int i=0; i<2; ++i){
                    //arms.points[i*2+1].old_pos = math.lerp(arms.points[i*2+1].old_pos, arms.points[i*2+1].pos - simple_vel*step, 0.75f);
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
                    float force = 50f;
                    var dir = math.lerp(math.normalize(jump_point - jump.target_com), math.normalize(predicted_land_point - jump.target_com), (Time.time - jump_time) / (predicted_land_time - jump_time));
                    arms.points[4].pos += dir * step_sqrd * force;
                    arms.points[0].pos -= dir * step_sqrd * force * 0.5f;
                    arms.points[2].pos -= dir * step_sqrd * force * 0.5f;
                    arms.points[0].pos[2] -= step_sqrd * simple_vel[0] * 1.0f;
                    arms.points[2].pos[2] += step_sqrd * simple_vel[0] * 1.0f;
                    arms.points[4].pos[0] -= simple_vel[0] * step_sqrd * 1f; // Apply backwards force to maintain forwards tilt
                                        
                    for(int i=0; i<2; ++i){
                        arms.bones[i].length[1] = arms.bones[i].length[0] / 0.4f * 0.8f;
                    }

                    arms.Constraints();
                }
                arms.EndSim();
                
                var up = math.normalize((arms.points[0].pos+arms.points[2].pos)*0.5f-arms.points[4].pos);
                var forward = math.normalize(math.cross(arms.points[0].pos - arms.points[2].pos, arms.points[0].pos - arms.points[4].pos));
                for(int i=0; i<2; ++i){
                    jump.limb_targets[2+i] = arms.points[i*2].pos - up + (up * 0.5f + forward * 0.1f) * dist_from_land;
                }

            }
            }
        }

        bool calc_walk = true;
        if(calc_walk){
            gallop_amount = math.clamp(math.abs(simple_vel[0]) / 4f + (math.sin(Time.time*0.7f)-1.0f) * 0.7f, 0.0f, 1.0f);
            quad_amount = math.clamp((math.sin(Time.time*2.3f)+math.sin(Time.time*1.7f)), 0.0f, 1.0f);

            var walk_lean = math.sin(Time.time)*0.2f+0.3f;
            var gallop_lean = math.sin(Time.time)*0.2f+0.8f + quad_amount * 0.07f * math.abs(effective_vel[0]);
            lean = math.lerp(walk_lean, gallop_lean, gallop_amount);

            float speed_mult = 8f/(math.PI*2f) * math.pow((math.abs(effective_vel[0])+1.0f),0.4f);
            walk_time += step*speed_mult;
            
            float target_skate_amount = 0.0f;
            if(slope_vec[1] < -0.5f && math.abs(effective_vel[0]) > 3.0f){
                target_skate_amount = 1.0f;
            }
            skate_amount = Mathf.MoveTowards(skate_amount, target_skate_amount, step*3.0f);
            quad_amount = math.lerp(quad_amount,0.0f,skate_amount);

            walk.body_compress_amount = math.lerp((math.sin((walk_time+quad_gallop_body_compress_offset) * math.PI * 2.0f)+1.0f)*quad_gallop_body_compress_amount*quad_amount*gallop_amount,
                                                  0.1f,
                                                  skate_amount);

            var target_com = simple_pos;
            target_com[1] = new_pos[1];
            float crouch_amount = 1.0f-climb_amount;
            var walk_height = math.lerp(base_walk_height, 0.3f, crouch_amount) + math.sin((walk_time+0.25f) * math.PI * 4.0f) * math.abs(effective_vel[0]) * 0.015f / speed_mult + math.abs(effective_vel[0])*0.01f;
            var gallop_height_ = math.sin((walk_time + gallop_height_offset) * math.PI * 2.0f) * gallop_height * math.abs(effective_vel[0]) + gallop_height_base;
            target_com[1] += math.lerp(walk_height, gallop_height_, gallop_amount);
            target_com[1] = math.lerp(target_com[1], simple_pos[1] + 0.5f, skate_amount);
            target_com[1] = math.lerp(target_com[1], simple_pos[1], math.abs(lean)*0.15f);
            
            var left = simple_pos - new float3(0.1f,0.0f,0.0f);
            var right = simple_pos + new float3(0.1f,0.0f,0.0f);
            left[1] = BranchesHeight(left[0]);
            right[1] = BranchesHeight(right[0]);
            float3 move_dir = math.normalize(right - left);

            if(in_air){
                walk.arms.StartSim(step);
                for(int i=0; i<display.arms.points.Count; ++i){
                    walk.arms.points[i].pos = display.arms.points[i].pos;
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
                        if(i!=1 && i!=3){
                            arms.points[i].pos += offset * 0.2f;
                        }
                    }
                    // Apply torque to keep torso upright and forward-facing
                    float step_sqrd = step*step;
                    float force = 20f;
                    var forward = math.normalize(math.cross(arms.points[0].pos - arms.points[2].pos, arms.points[0].pos - arms.points[4].pos));
                    var flat_forward = math.normalize(new float3(forward[0],0,forward[2]));
                    float3 top_force = (lean*flat_forward + new float3(0,1,0)) * force;
                    arms.points[4].pos += step_sqrd * -top_force;
                    arms.points[0].pos += step_sqrd * top_force * 0.5f;
                    arms.points[2].pos += step_sqrd * top_force * 0.5f;
                    arms.points[0].pos[2] -= step_sqrd * effective_vel[0] * 2.0f * (1.0f - skate_amount);
                    arms.points[2].pos[2] += step_sqrd * effective_vel[0] * 2.0f * (1.0f - skate_amount);
                    
                    
                    for(int i=0; i<2; ++i){
                        var walk_rotate = (math.cos((walk_time + tilt_offset) * math.PI * 2.0f + math.PI*i))*0.2f;
                        var gallop_rotate = (math.cos(math.PI*i))*(gallop_hip_rotate * (1.0f - quad_amount));
                        var rotate = math.lerp(walk_rotate, gallop_rotate, gallop_amount);
                        arms.points[i*2].pos[0] += step_sqrd * -3.0f * rotate * effective_vel[0] / speed_mult;
                    }
                    
                    // Move arms out to sides
                    float speed = math.abs(effective_vel[0])/max_speed;
                    for(int i=0; i<2; ++i){
                        arms_up = math.abs(speed * (math.sin(Time.time * ((i==1)?2.5f:2.3f))*0.3f+0.7f)) * (1.0f - gallop_amount);
                        arms.points[1+i*2].pos += step_sqrd * (arms.points[0].pos - arms.points[2].pos) * (1.5f+speed*2.0f+arms_up*2.0f) * (1-i*2) * 2f;
                        arms.points[1+i*2].pos[1] += step_sqrd * 10.0f * arms_up  * arms_up;
                        arms.bones[i].length[1] = arms.bones[0].length[0] / 0.4f * math.lerp((math.lerp(0.95f, 0.8f, math.min(speed*0.25f, 1.0f) + math.sin(arms_up * math.PI)*0.1f)),1.0f,gallop_amount);
                    }

                    // Make sure arms don't cross body
                    for(int i=0; i<2; ++i){
                        var side_dir = math.normalize(arms.points[0].pos - arms.points[2].pos) * (1-i*2);
                        float shoulder_d = math.dot(arms.points[i*2].pos, side_dir);
                        float hand_d = math.dot(arms.points[i*2+1].pos, side_dir);
                        float new_d = math.max(hand_d, shoulder_d);
                        arms.points[i*2+1].pos += (new_d - hand_d) * side_dir;
                    }

                    if(climb_amount < 1.0f && !wants_to_swing){
                        arms.points[1].pos = math.lerp(arms.points[1].pos, swing.arms.points[1].pos, (1.0f - climb_amount)*0.2f);
                        arms.points[3].pos = math.lerp(arms.points[3].pos, swing.arms.points[3].pos, (1.0f - climb_amount)*0.2f);
                    }
                    
                    for(int i=0; i<2; ++i){
                        arms.points[i*2+1].pos[1] = math.max(arms.points[i*2+1].pos[1], BranchesHeight(arms.points[i*2+1].pos[0]));
                    }
                    
                    if(gallop_amount * quad_amount > 0.0f){
                        for(int i=0; i<2; ++i){
                            walk.limb_targets[i] = arms.points[i*2].pos;
                            walk.limb_targets[i][1] = BranchesHeight(display.limb_targets[i][0]);
                            float time_val = walk_time * math.PI * 2.0f;// + math.PI*i*gallop_offset;
                            walk.limb_targets[i][1] += (-math.sin(time_val) + 0.5f)*gallop_arm_stride_height;
                            walk.limb_targets[i] += move_dir * (math.cos(time_val)+0.5f)*gallop_arm_stride * effective_vel[0] / speed_mult;
                            //arms.points[i*2+1].pos = math.lerp(arms.points[i*2+1].pos, limb_targets[i], 0.01f);
                            arms.points[i*2+1].pos = MoveTowards(arms.points[i*2+1].pos, walk.limb_targets[i], step * 0.5f * gallop_amount * quad_amount);
                    
                            //DebugDraw.Sphere(walk.limb_targets[i], Color.green, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray);
                        }
                    }

                    for(int i=0; i<2; ++i){
                        arms.Constraints();
                    }
                }
                arms.EndSim();
                for(int i=0; i<2; ++i){
                    var offset = math.lerp(gallop_offset, quad_gallop_offset, quad_amount);
                    float time_val = walk_time * math.PI * 2.0f + math.PI*i*offset;
                    walk.limb_targets[2+i] = simple_pos;
                    walk.limb_targets[2+i] += (move_dir * (math.cos(walk_time * math.PI * 2.0f + math.PI*i))*0.2f - 0.03f) * effective_vel[0] / speed_mult  * (1.0f - skate_amount) * (1.0f-gallop_amount);
                    walk.limb_targets[2+i] += (move_dir * (math.cos(time_val))*0.2f - 0.03f) * effective_vel[0] / speed_mult  * (1.0f - skate_amount) * gallop_stride  * (1.0f - skate_amount) * gallop_amount;
                    walk.limb_targets[2+i] += (arms.points[0].pos - arms.points[2].pos) * (1.0f-2.0f*i) * (0.3f+0.3f*skate_amount);
                    walk.limb_targets[2+i][1] = BranchesHeight(walk.limb_targets[2+i][0]); 
                    walk.limb_targets[2+i][1] += (-math.sin(walk_time * math.PI * 2.0f + math.PI*i) + 1.0f)*0.2f * (math.pow(math.abs(effective_vel[0]) + 1.0f, 0.3f) - 1.0f) * (1.0f - skate_amount) * (1.0f-gallop_amount);
                    walk.limb_targets[2+i][1] += (-math.sin(time_val) + 1.0f)*gallop_stride_height * (math.pow(math.abs(effective_vel[0]) + 1.0f, 0.3f) - 1.0f) * (1.0f - skate_amount) * gallop_amount; 
                }
            }
        } 
        
        {
            var old_com = float3.zero;
            {
                float total_mass = 0.0f;
                var points = display.arms.points;
                for(int i=0; i<points.Count; ++i){
                    if(points[i].pinned == false){
                        old_com += points[i].pos * points[i].mass;
                        total_mass += points[i].mass;
                    }
                }
                old_com /= total_mass;
            }

            for(int i=0; i<display.arms.points.Count; ++i){
                display.arms.points[i].pos = math.lerp(swing.arms.points[i].pos, walk.arms.points[i].pos, climb_amount);
                display.arms.points[i].pos = math.lerp(display.arms.points[i].pos, jump.arms.points[i].pos, in_air_amount);
            }
            display.arms.Constraints();
            for(int i=0; i<4; ++i){
                display.limb_targets[i] = math.lerp(swing.limb_targets[i], walk.limb_targets[i], climb_amount);
                display.limb_targets[i] = math.lerp(display.limb_targets[i], jump.limb_targets[i], in_air_amount);
            }
            body_compress_amount = math.lerp(swing.body_compress_amount, walk.body_compress_amount, climb_amount);
            body_compress_amount = math.lerp(body_compress_amount, jump.body_compress_amount, in_air_amount);

            var com = float3.zero;
            {
                float total_mass = 0.0f;
                var points = display.arms.points;
                for(int i=0; i<points.Count; ++i){
                    if(points[i].pinned == false){
                        com += points[i].pos * points[i].mass;
                        total_mass += points[i].mass;
                    }
                }
                com /= total_mass;
            }
            //DebugDraw.Line(old_com, com, Color.green, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
        }

        // Move game camera to track character
        var cam_pos = Camera.main.transform.position;
        cam_pos[0] = simple_pos[0];
        
        {
            float total_mass = 0.0f;
            var com = new float3(0.0f, 0.0f, 0.0f);
            var points = display.arms.points;
            for(int i=0; i<points.Count; ++i){
                com += points[i].pos * points[i].mass;
                total_mass += points[i].mass;
            }
            com /= total_mass;
            cam_pos[0] = com[0] + simple_vel[0] * 0.1f;
        }

        Camera.main.transform.position = cam_pos;
    }

    private void FixedUpdate() {
        Step(Time.fixedDeltaTime);
    }
}
}