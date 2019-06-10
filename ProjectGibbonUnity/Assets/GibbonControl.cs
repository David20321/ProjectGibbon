using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GibbonControl : MonoBehaviour
{
    Vector3 vel;
    Vector3 last_pos;
    Vector3 pos;
    Vector3[] hand_pos;
    int next_hand = 0;

    // Start is called before the first frame update
    void Start()
    {
        pos = Vector3.zero;
        hand_pos = new Vector3[] {Vector3.zero, Vector3.zero};
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
        DebugDraw.Line(hand_pos[0], pos, Color.green, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Normal);
        DebugDraw.Line(hand_pos[1], pos, Color.blue, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Normal);
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
        if(Input.GetKeyDown(KeyCode.Space)){
            hand_pos[next_hand] = hand_pos[1-next_hand] + Vector3.right;
            next_hand = 1-next_hand;
        }
        pos = pos + (pos - last_pos) + acc * Time.fixedDeltaTime * Time.fixedDeltaTime;
        float arm_length = 0.8f;
        for(int i=0; i<2; ++i){
            if(Vector3.Distance(hand_pos[i], pos) > arm_length){
                pos = Vector3.Normalize(pos - hand_pos[i]) * arm_length + hand_pos[i];
            }
        }
        last_pos = temp_pos;
    }
}
