using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
#if UNITY_EDITOR
using UnityEditor;

public class ScriptOrder:Attribute {
    public int order;
 
    public ScriptOrder(int order) {
        this.order = order;
    }
}

 [InitializeOnLoad]
public class ScriptOrderManager {
 
    static ScriptOrderManager() {
        foreach (MonoScript monoScript in MonoImporter.GetAllRuntimeMonoScripts()) {
            if (monoScript.GetClass() != null) {
                foreach (var a in Attribute.GetCustomAttributes(monoScript.GetClass(), typeof(ScriptOrder))) {
                    var currentOrder = MonoImporter.GetExecutionOrder(monoScript);
                    var newOrder = ((ScriptOrder)a).order;
                    if (currentOrder != newOrder)
                        MonoImporter.SetExecutionOrder(monoScript, newOrder);
                }
            }
        }
    }
}

[ScriptOrder(-100)]
#endif
public class DebugDraw : MonoBehaviour {
    public GameObject translucent_sphere_prefab;
    static DebugDraw instance;
    public Material line_material;
    public Material line_material_xray;
    int color_property_id;

    class LineCollection {
        public Mesh mesh = null;
        public GameObject mesh_object = null;
        public MeshRenderer mesh_renderer = null;
        public MeshFilter mesh_filter = null;
        public List<DebugDrawLine> lines = new List<DebugDrawLine>();

        public void Initialize(string obj_name, Material material) {
            mesh_object = new GameObject("DebugDrawLineMesh");
            mesh = new Mesh();
            mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
            mesh_renderer = mesh_object.AddComponent<MeshRenderer>();
            mesh_filter = mesh_object.AddComponent<MeshFilter>();
            mesh_filter.mesh = mesh;
            mesh_renderer.material = material;
        }

        static List<Vector3> vertices = new List<Vector3>();
        static List<int> indices = new List<int>();
        static List<Color> colors = new List<Color>();
        static int[] null_array = new int[0];

        public void StartFrame() {
            if(mesh_renderer != null) {
                if (lines.Count > 0) {
                    mesh_renderer.enabled = true;
                    vertices.Clear();
                    indices.Clear();
                    colors.Clear();
                    foreach (DebugDrawLine line in lines) {
                        indices.Add(vertices.Count);
                        vertices.Add(line.start);
                        colors.Add(line.start_color);
                        indices.Add(vertices.Count);
                        vertices.Add(line.end);
                        colors.Add(line.end_color);
                    }
                    mesh.SetIndices(null_array, MeshTopology.Lines, 0);
                    mesh.SetVertices(vertices);
                    mesh.SetIndices(indices.ToArray(), MeshTopology.Lines, 0);
                    mesh.SetColors(colors);
                    mesh.RecalculateBounds();
                } else {
                    mesh_renderer.enabled = false;
                }
            }
        }

        public void EndFrame() {
            for (int i = lines.Count - 1; i >= 0; --i) {
                if (lines[i].lifetime == Lifetime.OneFrame) {
                    lines.RemoveAt(i);
                }
            }
        }

        public void EndFixedFrame() {
            for (int i = lines.Count - 1; i >= 0; --i) {
                if (lines[i].lifetime == Lifetime.OneFixedUpdate) {
                    lines.RemoveAt(i);
                }
            }
        }
    }
    
    class SphereCollection {
        public Material material;
        public List<DebugDrawWireSphere> spheres = new List<DebugDrawWireSphere>();

        public void StartFrame() {
            foreach (DebugDrawWireSphere sphere in spheres) {
                Matrix4x4 matrix = new Matrix4x4();
                matrix.SetTRS(sphere.center, sphere.rotation, sphere.scale);
                MaterialPropertyBlock material_property_block = new MaterialPropertyBlock();
                material_property_block.SetColor(instance.color_property_id, sphere.color);
                Graphics.DrawMesh(instance.sphere_mesh, matrix, material, instance.translucent_sphere_prefab.layer, null, 0, material_property_block);
            }
        }

        public void EndFrame() {
            for (int i = spheres.Count - 1; i >= 0; --i) {
                if (spheres[i].lifetime == Lifetime.OneFrame) {
                    spheres.RemoveAt(i);
                }
            }
        }

        public void EndFixedFrame() {
            for (int i = spheres.Count - 1; i >= 0; --i) {
                if (spheres[i].lifetime == Lifetime.OneFixedUpdate) {
                    spheres.RemoveAt(i);
                }
            }
        }
    }

    Mesh sphere_mesh = null;
    
    public enum Lifetime { Persistent, OneFrame, OneFixedUpdate };
    public enum Type { Normal, Xray };

    public class DebugDrawLine {
        public Vector3 start, end;
        public Color start_color, end_color;
        public Lifetime lifetime;
    }

    public class DebugDrawWireSphere {
        public Vector3 center, scale;
        public Quaternion rotation;
        public Color color;
        public Lifetime lifetime;
    }

    LineCollection lines      = new LineCollection();
    LineCollection lines_xray = new LineCollection();
    SphereCollection spheres = new SphereCollection();
    SphereCollection spheres_xray = new SphereCollection();

    public static void Remove(DebugDrawLine to_remove) {
        instance.lines.lines.Remove(to_remove);
        instance.lines_xray.lines.Remove(to_remove);
    }

    public static void Remove(DebugDrawWireSphere to_remove) {
        instance.spheres.spheres.Remove(to_remove);
        instance.spheres_xray.spheres.Remove(to_remove);
    }

    void Awake() {
        instance = this;

        lines.Initialize("DebugDrawLineMesh", line_material);
        lines_xray.Initialize("DebugDrawLineMeshXRay", line_material_xray);
        spheres.material = line_material;
        spheres_xray.material = line_material_xray;
        sphere_mesh = translucent_sphere_prefab.GetComponent<MeshFilter>().sharedMesh;

        color_property_id = Shader.PropertyToID("_TintColor");
        
    }

    public static DebugDrawWireSphere Sphere(Vector3 center, Color color, Vector3 scale, Quaternion rotation, Lifetime lifetime, Type type) {
        DebugDrawWireSphere sphere = new DebugDrawWireSphere();
        sphere.center = center;
        sphere.color = color;
        sphere.scale = scale;
        sphere.rotation = rotation;
        sphere.lifetime = lifetime;
        if (type == Type.Xray) {
            instance.spheres_xray.spheres.Add(sphere);
        } else {
            instance.spheres.spheres.Add(sphere);
        }
        return sphere;
    }
    
    public static DebugDrawLine Line(Vector3 start, Vector3 end, Color color, Lifetime lifetime, Type type) {
        return Line(start, end, color, color, lifetime, type);
    }

    public static DebugDrawLine Line(Vector3 start, Vector3 end, Color start_color, Color end_color, Lifetime lifetime, Type type) {
        DebugDrawLine line = new DebugDrawLine();
        line.start = start;
        line.end = end;
        line.start_color = start_color;
        line.end_color = end_color;
        line.lifetime = lifetime;
        if(type == Type.Xray) {
            instance.lines_xray.lines.Add(line);
        } else {
            instance.lines.lines.Add(line);
        }
        return line;
    }

    public static void Box(Vector3 center, Color color, Vector3 scale, Quaternion rotation, Lifetime lifetime, Type type) {
        Vector3 corner = scale * 0.5f;
        // Top
        Line(center + rotation * Vector3.Scale(corner, new Vector3(-1f, 1f, 1f)),
             center + rotation * Vector3.Scale(corner, new Vector3( 1f, 1f, 1f)), 
             color, lifetime, type);
        Line(center + rotation * Vector3.Scale(corner, new Vector3( 1f, 1f, 1f)),
             center + rotation * Vector3.Scale(corner, new Vector3( 1f, 1f,-1f)), 
             color, lifetime, type);
        Line(center + rotation * Vector3.Scale(corner, new Vector3( 1f, 1f,-1f)),
             center + rotation * Vector3.Scale(corner, new Vector3(-1f, 1f,-1f)), 
             color, lifetime, type);
        Line(center + rotation * Vector3.Scale(corner, new Vector3(-1f, 1f,-1f)),
             center + rotation * Vector3.Scale(corner, new Vector3(-1f, 1f, 1f)), 
             color, lifetime, type);
        // Bottom
        Line(center + rotation * Vector3.Scale(corner, new Vector3(-1f,-1f, 1f)),
             center + rotation * Vector3.Scale(corner, new Vector3( 1f,-1f, 1f)), 
             color, lifetime, type);
        Line(center + rotation * Vector3.Scale(corner, new Vector3( 1f,-1f, 1f)),
             center + rotation * Vector3.Scale(corner, new Vector3( 1f,-1f,-1f)), 
             color, lifetime, type);
        Line(center + rotation * Vector3.Scale(corner, new Vector3( 1f,-1f,-1f)),
             center + rotation * Vector3.Scale(corner, new Vector3(-1f,-1f,-1f)), 
             color, lifetime, type);
        Line(center + rotation * Vector3.Scale(corner, new Vector3(-1f,-1f,-1f)),
             center + rotation * Vector3.Scale(corner, new Vector3(-1f,-1f, 1f)), 
             color, lifetime, type);
        // Connectors
        Line(center + rotation * Vector3.Scale(corner, new Vector3( 1f,-1f, 1f)),
             center + rotation * Vector3.Scale(corner, new Vector3( 1f, 1f, 1f)), 
             color, lifetime, type);
        Line(center + rotation * Vector3.Scale(corner, new Vector3(-1f,-1f, 1f)),
             center + rotation * Vector3.Scale(corner, new Vector3(-1f, 1f, 1f)), 
             color, lifetime, type);
        Line(center + rotation * Vector3.Scale(corner, new Vector3(-1f,-1f,-1f)),
             center + rotation * Vector3.Scale(corner, new Vector3(-1f, 1f,-1f)), 
             color, lifetime, type);
        Line(center + rotation * Vector3.Scale(corner, new Vector3( 1f,-1f,-1f)),
             center + rotation * Vector3.Scale(corner, new Vector3( 1f, 1f,-1f)), 
             color, lifetime, type);
    }

    WaitForEndOfFrame wait_for_end_of_frame = new WaitForEndOfFrame();

    IEnumerator Clear() {
        yield return wait_for_end_of_frame;
        lines.EndFrame();
        lines_xray.EndFrame();
        spheres.EndFrame();
        spheres_xray.EndFrame();
    }
    
    void LateUpdate() {
        lines.StartFrame();
        lines_xray.StartFrame();
        spheres.StartFrame();
        spheres_xray.StartFrame();
        StartCoroutine("Clear");
    }

    void FixedUpdate() {
        lines.EndFixedFrame();
        lines_xray.EndFixedFrame();
        spheres.EndFixedFrame();
        spheres_xray.EndFixedFrame();
    }
}
