using ImGuiNET;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace Wolfire {

public struct MyTransform {
    public Vector3 pos;
    public Quaternion rot;
    public static MyTransform fromTransform(Transform transform){
        var ret_val = new MyTransform();
        ret_val.pos = transform.position;
        ret_val.rot = transform.rotation;
        return ret_val;
    }
    public void toTransform(Transform transform){
        transform.position = pos;
        transform.rotation = rot;
    }
};

public class Pose {
    public enum Part {rig, body, chest, hand_l, hand_r, foot_l, foot_r, num_transforms};        
    public MyTransform[] transforms = new MyTransform[(int)Part.num_transforms];
    public string name;
}

public class AnimationEditor : MonoBehaviour {
    string temp_name = "animations";
    string temp_file_path;
    int version = 1;
    public GameObject rig;
    
    public List<Pose> poses = new List<Pose>();

    void LoadFromFile(string path) {
        Debug.Log($"Found file {path}");
        using (FileStream file = File.Open(path, FileMode.Open, FileAccess.Read)) {
            using (BinaryReader bw = new BinaryReader(file)) {
                Debug.Log($"Reading");
                int version = bw.ReadInt32();
                int num_poses = bw.ReadInt32();
                poses.Clear();
                for(int i=0; i<num_poses; ++i){
                    var pose = new Pose();
                    ReadPose(bw, pose);
                    poses.Add(pose);
                }
                Debug.Log($"Read animation version {version}");
                //terrain.Deserialize(data);
            }
        }
    }
    

    void TestCompare(string a, string b) {
        a = a.Trim();
        if (a.Length != b.Length)
            throw new Exception("Well here's the problem");

        for (int i = 0; i < a.Length; i++) {
            if (a[i] != b[i]) {
                throw new Exception("Difference at character: " + i + 1);
            }
        }
    }

    // Start is called before the first frame update
    void Start() {
        temp_file_path = $"{Application.persistentDataPath}{Path.DirectorySeparatorChar}{temp_name}";
        var app_path = $"{Application.dataPath}{Path.DirectorySeparatorChar}{temp_name}";
        if (File.Exists(temp_file_path)) {
            LoadFromFile(temp_file_path);
        } else if (File.Exists(app_path)) {
            LoadFromFile(app_path);
        }
        TestCompare(poses[2].name, "Hang2");
    }

    void WriteQuaternion(BinaryWriter bw, Quaternion quat) {
        for(int i=0; i<4; ++i){
            bw.Write(quat[i]);
        }
    }
    
    void WriteVector3(BinaryWriter bw, Vector3 vec) {
        for(int i=0; i<3; ++i){
            bw.Write(vec[i]);
        }
    }

    void WriteTransform(BinaryWriter bw, MyTransform transform) {
        WriteVector3(bw, transform.pos);
        WriteQuaternion(bw, transform.rot);
    }
    
    void ReadQuaternion(BinaryReader bw, out Quaternion quat) {
        quat = Quaternion.identity;
        for(int i=0; i<4; ++i){
            quat[i] = bw.ReadSingle();
        }
    }
    
    void ReadVector3(BinaryReader bw, out Vector3 vec) {
        vec = Vector3.zero;
        for(int i=0; i<3; ++i){
            vec[i] = bw.ReadSingle();
        }
    }

    void ReadTransform(BinaryReader bw, ref MyTransform transform) {
        ReadVector3(bw, out Vector3 pos);
        ReadQuaternion(bw, out Quaternion rot);
        transform.pos = pos;
        transform.rot = rot;
    }

    void ReadPose(BinaryReader bw, Pose pose) {
        pose.name = bw.ReadString();
        for(int i=0; i<pose.transforms.Length; ++i){
            ReadTransform(bw, ref pose.transforms[i]);
        }
    }
    
    void WritePose(BinaryWriter bw, Pose pose) {
        bw.Write(pose.name);
        for(int i=0; i<pose.transforms.Length; ++i){
            WriteTransform(bw, pose.transforms[i]);
        }
    }

    void SaveAnimation() {
        Debug.Log($"Serializing");
        //var data = terrain.Serialize();

        Debug.Log($"Writing file: {temp_file_path}");
        using (FileStream file = File.Open(temp_file_path, FileMode.Create, FileAccess.Write)) {
            using (BinaryWriter bw = new BinaryWriter(file)) {
                bw.Write(version);
                bw.Write(poses.Count);
                foreach(var pose in poses){
                    WritePose(bw, pose);
                }
                Debug.Log($"Done writing");
            }
        }
    }
    
    char[] str0 = "".PadRight(128, '\0').ToCharArray();
    int renaming = -1;

    public static void ApplyPose(Transform rig_transform, Pose pose){
        pose.transforms[(int)Pose.Part.rig].toTransform(rig_transform);
        pose.transforms[(int)Pose.Part.body].toTransform(rig_transform.Find("body_ik"));
        pose.transforms[(int)Pose.Part.chest].toTransform(rig_transform.Find("chest_and_mesh"));
        pose.transforms[(int)Pose.Part.foot_l].toTransform(rig_transform.Find("foot_ik_l"));
        pose.transforms[(int)Pose.Part.foot_r].toTransform(rig_transform.Find("foot_ik_r"));
        pose.transforms[(int)Pose.Part.hand_l].toTransform(rig_transform.Find("hand_ik_l"));
        pose.transforms[(int)Pose.Part.hand_r].toTransform(rig_transform.Find("hand_ik_r"));    
    }

    private void Update() {
        if(ImGui.Begin("Editor")){
            if(ImGui.Button("Save Pose")){
                var pose = new Pose();
                var rig_transform = rig.transform;
                pose.transforms[(int)Pose.Part.rig] = MyTransform.fromTransform(rig_transform);
                pose.transforms[(int)Pose.Part.body] = MyTransform.fromTransform(rig_transform.Find("body_ik"));
                pose.transforms[(int)Pose.Part.chest] = MyTransform.fromTransform(rig_transform.Find("chest_and_mesh"));
                pose.transforms[(int)Pose.Part.foot_l] = MyTransform.fromTransform(rig_transform.Find("foot_ik_l"));
                pose.transforms[(int)Pose.Part.foot_r] = MyTransform.fromTransform(rig_transform.Find("foot_ik_r"));
                pose.transforms[(int)Pose.Part.hand_l] = MyTransform.fromTransform(rig_transform.Find("hand_ik_l"));
                pose.transforms[(int)Pose.Part.hand_r] = MyTransform.fromTransform(rig_transform.Find("hand_ik_r"));
                pose.name = "Unnamed pose";
                poses.Add(pose);
            }
            int index=0;
            int to_delete = -1;
            foreach(var pose in poses){
                if(ImGui.Button($"Apply##{index}")){
                    ApplyPose(rig.transform, pose);
                }
                ImGui.SameLine();
                if(renaming != index && ImGui.Button(pose.name)){
                    renaming = index;
                    str0 = pose.name.PadRight(128, '\0').ToCharArray();
                } else if(renaming == index){
                    if(ImGui.InputText("", str0, ImGuiInputTextFlags.EnterReturnsTrue)){
                        int len = 0;
                        for(int i=0; i<128; ++i){
                            if(str0[i] == '\0'){
                                len = i;
                                break;
                            }
                        }
                        pose.name = new string(str0, 0, len);
                        renaming = -1;
                    }
                }
                ImGui.SameLine();
                if(ImGui.Button($"Delete##{index}")){
                   to_delete = index; 
                }
                ++index;
            }
            if(to_delete != -1){
                poses.RemoveAt(to_delete);
            }
        }
        ImGui.End();
    }

    private void OnApplicationQuit() {
        SaveAnimation();
    }
}
}