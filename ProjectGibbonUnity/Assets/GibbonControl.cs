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
    public GameObject display_gibbon; // Character mesh and bone transforms
    public GameObject point_prefab; // Widget for manipulating point positions
    
    // Current target position for each hand
    class HandState {
        public float3 pos;
        public bool gripping;
    }
    HandState[] hands;
    int next_hand = 1; // Which hand to grip with next
    
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
    Verlet.System arms =     new Verlet.System(); // For swinging
    Verlet.System complete = new Verlet.System(); // For final animation
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
    float3 target_com;
    float3 simple_vel = float3.zero;
    float swing_time = 0f;

    void Start() {
        Verlet.point_prefab_static = point_prefab;

        // Starting point
        simple_pos = display_gibbon.transform.position;
        simple_pos[1] = 0f;
        simple_pos[2] = 0f;

        // Init hand positions
        hands = new HandState[2];
        for(int i=0; i<2; ++i){
            hands[i] = new HandState();
            hands[i].pos = simple_pos;
            hands[i].gripping = true;
        }
        hands[1].gripping = false;

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

        // Set up particles and bones for swinging sim
        arms.AddPoint(shoulder.position, "shoulder_r");
        arms.AddPoint(grip.position, "hand_r");
        arms.AddPoint((shoulder.position+Vector3.right * (neck.position[0] - shoulder.position[0])*2f), "shoulder_l");
        arms.AddPoint((grip.position+Vector3.right * (neck.position[0] - grip.position[0])*2f), "hand_l");
        arms.AddPoint(new float3(neck.position[0], hip.position[1], neck.position[2]), "body");
        arms.points[0].mass = 2f;
        arms.points[2].mass = 2f;
        arms.points[4].mass = 4f;
        
        float measured_arm_length = Vector3.Distance(shoulder.position, elbow.position) + Vector3.Distance(elbow.position, grip.position);
        arms.AddBone("arm_r", 0, 1);
        arms.bones[arms.bones.Count-1].length[1] = measured_arm_length;
        arms.bones[arms.bones.Count-1].length[0] *= 0.4f; // Allow arm to flex
        arms.AddBone("arm_l", 2, 3);
        arms.bones[arms.bones.Count-1].length[1] = measured_arm_length;
        arms.bones[arms.bones.Count-1].length[0] *= 0.4f;
        arms.AddBone("tri_top", 0, 2);
        arms.AddBone("tri_r", 0, 4);
        arms.AddBone("tri_l", 2, 4);
        
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
        
        branches.AddPoint(new float3(0,0,0), "branch");
        branches.AddPoint(new float3(10,4,0), "branch");
        branches.AddBone("branch", 0, 1);
        branches.points[0].pinned = true;
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

            // Leg motion
            for(int i=0; i<2; ++i){
                complete.points[11+i*2].pos += up * (0.35f + 0.15f * math.sin((swing_time + i*1.0f)*math.PI));
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

        if(ImGui.Begin("Gibbon")){
            ImGui.Text($"horz speed: {math.abs(simple_vel[0])}");  
            ImGui.Text($"swing time: {swing_time}");  
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
                
        // Simple velocity control
        var old_pos = simple_pos;
        simple_vel[0] += horz_input * Time.deltaTime * 5f;
        simple_vel[0] = math.clamp(simple_vel[0], -10f, 10f);
        simple_pos += simple_vel * Time.deltaTime;

        // Adjust amplitude and time scale based on speed
        float amplitude = math.pow(math.abs(simple_vel[0])/10f + 1f, 0.8f)-1f+0.1f;
        float min_height = -1f + amplitude * 0.25f + math.max(0.0f, 0.1f - math.abs(simple_vel[0]) * 0.1f);
        var old_swing_time = swing_time;
        float swing_speed_mult = 8f/(math.PI*2f);
        swing_time += step*swing_speed_mult;
        if(math.ceil(old_swing_time) != math.ceil(swing_time)){
            next_hand = 1-next_hand;
        }
        var pendulum_length = 0.9f;
        simple_pos[1] = (min_height + (math.sin((swing_time-0.1f) * (math.PI*2f))+1f)*amplitude) * pendulum_length;
        
        //DebugDraw.Sphere(simple_pos, Color.green, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray );
        //DebugDraw.Line(old_pos, simple_pos, Color.green, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray );    
        
        // Branches
        float branch_t = (simple_pos[0]-branches.points[0].bind_pos[0])/(branches.points[1].bind_pos[0]-branches.points[0].bind_pos[0]);
        DebugDraw.Sphere(math.lerp(branches.points[0].pos, branches.points[1].pos, branch_t), Color.green, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray );
        
        /*branches.StartSim(step);
        branches.points[1].pos[1] -= branch_t * step * step * 100f * math.max(0,math.cos(swing_time*math.PI*2.0f));
        branches.points[1].pos = math.lerp(branches.points[1].pos, branches.points[1].bind_pos, 0.1f);
        branches.Constraints();
        branches.EndSim();
        */

        simple_pos[1] += BranchHeight(simple_pos[0],0,1);

        target_com = simple_pos - simple_vel * 0.05f;
        target_com[0] += (math.cos((swing_time-0.1f) * (math.PI*2f)))* pendulum_length * 0.5f * math.clamp(simple_vel[0] * 0.5f, -1f, 1f) * math.max(0f, 1f - math.abs(simple_vel[0])*2f);
        
        // Figure out target hand positions
        float next_trough_time = ((math.ceil(swing_time)-0.25f))/swing_speed_mult;
        hands[next_hand].pos = simple_pos + simple_vel * (next_trough_time-Time.time);
        for(int i=0; i<2; ++i){
            hands[i].pos[1] = BranchHeight(hands[i].pos[0],0,1);
        }
        DebugDraw.Sphere(hands[0].pos, Color.green, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray );
        DebugDraw.Sphere(hands[1].pos, Color.blue, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFixedUpdate, DebugDraw.Type.Xray );
        
        // Use COM and hand positions to drive arm rig
        bool arms_map = true;
        if(arms_map){
            // Move hands towards grip targets
            for(int i=0; i<2; ++i){
                arms.points[i*2+1].pos = MoveTowards(arms.points[i*2+1].pos, hands[i].pos, math.max(0f, math.cos((swing_time+0.35f+(1-i))*math.PI*1f)*0.5f+0.5f) * step * 5f);
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
                var offset = (float3)target_com - com;
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