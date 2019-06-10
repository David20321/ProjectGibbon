using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GibbonControl : MonoBehaviour
{
    public GameObject gibbon;

    Vector3 vel;
    Vector3 last_pos;
    Vector3 pos;
    class HandState {
        public Vector3 pos;
        public bool gripping;
    }
    float last_x;

    HandState[] hands;
    int next_hand = 0;
    const float arm_length = 0.8f;

    // Start is called before the first frame update
    void Start()
    {
        pos = Vector3.zero;
        hands = new HandState[2];
        for(int i=0; i<2; ++i){
            hands[i] = new HandState();
            hands[i].pos = pos;
            hands[i].gripping = true;
        }
        hands[next_hand].gripping = false;
        
        var root = gibbon.transform.Find("rig/root");
        var hip = root.Find("DEF-spine");
        var belly = hip.Find("DEF-spine_001");
        var chest = belly.Find("DEF-spine_002/DEF-spine_003");
        var neck = chest.Find("DEF-spine_004/DEF-spine_005");
        var head = neck.Find("DEF-spine_006");
        var leg_root = root.Find("torso/MCH-spine_001/MCH-spine/tweak_spine/ORG-spine");
        var left_thigh = leg_root.Find("DEF-thigh_L");
        var left_knee = left_thigh.Find("DEF-thigh_L_001/DEF-shin_L");
        var left_foot = left_knee.Find("DEF-shin_L_001/DEF-foot_L");
        var arm_root = root.Find("torso/MCH-spine_002/MCH-spine_003/tweak_spine_003/ORG-spine_003");
        var left_clavicle = arm_root.Find("ORG-shoulder_L");
        var left_shoulder = left_clavicle.Find("DEF-upper_arm_L");
        var left_elbow = left_shoulder.Find("DEF-upper_arm_L_001/DEF-forearm_L");
        var left_wrist = left_elbow.Find("DEF-forearm_L_001/DEF-hand_L");
        
        DebugDraw.Line(neck.position, head.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
        DebugDraw.Line(neck.position, chest.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
        DebugDraw.Line(belly.position, chest.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
        DebugDraw.Line(belly.position, hip.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
        DebugDraw.Line(left_thigh.position, hip.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
        DebugDraw.Line(left_thigh.position, left_knee.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
        DebugDraw.Line(left_foot.position, left_knee.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
        DebugDraw.Line(left_foot.position, left_foot.TransformPoint(Vector3.right * -0.7f), Color.yellow, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
        DebugDraw.Line(left_clavicle.position, chest.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
        DebugDraw.Line(left_clavicle.position, left_shoulder.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
        DebugDraw.Line(left_elbow.position, left_shoulder.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
        DebugDraw.Line(left_elbow.position, left_wrist.position, Color.white, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
        DebugDraw.Line(left_wrist.position, left_wrist.TransformPoint(Vector3.right*-1.0f), Color.yellow, DebugDraw.Lifetime.Persistent, DebugDraw.Type.Xray);
        
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
        if(false){ // Walking on branch
            float max_speed = 4f;
            Vector3 target_vel = Vector3.zero;
            if(Input.GetKey(KeyCode.A)){
                target_vel = -Vector3.right * max_speed;
            }
            if(Input.GetKey(KeyCode.D)){
                target_vel = Vector3.right * max_speed;
            }
            vel = Vector3.Lerp(target_vel, vel, Mathf.Pow(0.01f, Time.deltaTime));
            transform.position += vel * Time.deltaTime;
        }

        DebugDraw.Sphere(pos, Color.red, Vector3.one * 0.25f, Quaternion.identity, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Normal);
        for(int i=0; i<2; ++i){
            if(hands[i].gripping){
                DebugDraw.Line(hands[i].pos, pos, i==0?Color.green:Color.blue, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Normal);
            }
        }

        int num_segments = 20;
        for(int i=1; i<num_segments+1; ++i){
            float interp = (i-1)/(float)num_segments * Mathf.PI;
            float interp2 = i/(float)num_segments * Mathf.PI;
            var temp_pos = pos + (Vector3.right * Mathf.Sin(interp) - Vector3.up * Mathf.Cos(interp))*arm_length;
            var temp_pos2 = pos + (Vector3.right * Mathf.Sin(interp2) - Vector3.up * Mathf.Cos(interp2))*arm_length;
            DebugDraw.Line(temp_pos, temp_pos2, new Color(1.0f, 0.0f, 0.0f, 1.0f), DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Normal);
        }
    }

    private void FixedUpdate() {
        var temp_pos = pos;
        var acc = Vector3.up * -9.81f;
        if(Input.GetKey(KeyCode.A)){
            acc -= Vector3.right;
        }
        if(Input.GetKey(KeyCode.D)){
            acc += Vector3.right;
        }
        
        float y = ((-pos[1])/arm_length);
        float x = Mathf.Sqrt(1.0f - y*y);
        if(Input.GetKey(KeyCode.D)){
            float potential_x = pos[0] + x * arm_length;
            if(x < last_x && potential_x > hands[1-next_hand].pos[0] + 0.01f){
                hands[next_hand].gripping = true;
                hands[next_hand].pos[0] = potential_x;
                next_hand = 1-next_hand;
                hands[next_hand].gripping = false;
            }
        } else if(Input.GetKey(KeyCode.A)){
            float potential_x = pos[0] - x * arm_length;
            if(x < last_x && potential_x < hands[1-next_hand].pos[0] - 0.01f){
                hands[next_hand].gripping = true;
                hands[next_hand].pos[0] = potential_x;
                next_hand = 1-next_hand;
                hands[next_hand].gripping = false;
            }
        }
        last_x = x;

        pos = pos + (pos - last_pos) + acc * Time.fixedDeltaTime * Time.fixedDeltaTime;
        for(int i=0; i<2; ++i){
            if(hands[i].gripping && Vector3.Distance(hands[i].pos, pos) > arm_length){
                pos = Vector3.Normalize(pos - hands[i].pos) * arm_length + hands[i].pos;
            }
        }
        last_pos = temp_pos;
    }
}
