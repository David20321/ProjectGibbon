using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ImGuiNET;

public class GibbonGame : MonoBehaviour
{
    class Surface {
        enum Type {SolidWalk, SolidSlide, WobbleGrab};
    }

    // Start is called before the first frame update
    void Start() {
        
    }

    Vector3 clicked_point = Vector3.zero;

    // Update is called once per frame
    void Update() {
        if(ImGui.Begin("Editor")){
            ImGui.Text("Right-click to select point");
            ImGui.Text("Ctrl-click to add new point");   
        }
        ImGui.End();
        
        if(Input.GetMouseButtonDown(0) && (Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl) ) ) {
            // Ctrl click
            var ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            var t = -ray.origin.z / ray.direction.z;
            clicked_point = ray.origin + ray.direction * t; 
        }

        DebugDraw.Sphere(clicked_point, Color.green, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Normal);
        DebugDraw.Line(new Vector3(-10,0,0), new Vector3(10,0,0), Color.red, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Normal);
    }
}
