using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PoseExtract : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        var animator = GetComponent<Animator>();
        var controller = animator.runtimeAnimatorController;
        var clips = controller.animationClips;
        foreach(var clip in clips){
            Debug.Log(clip.name);
            clip.SampleAnimation(gameObject, 0.0f);
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
