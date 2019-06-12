using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace RootMotion.FinalIK
{
    [CustomEditor(typeof(CCDIKJ))]
    public class CCDIKJInspector : Editor
    {
        private CCDIKJ script { get { return target as CCDIKJ; } }

        void OnSceneGUI()
        {
            // Draw the scene veiw helpers
            AddScene(script, Color.cyan, true);
        }

        // Draws the scene view helpers for IKSolverHeuristic
        public static void AddScene(CCDIKJ ik, Color color, bool modifiable)
        {
            // Protect from null reference errors
            if (Application.isPlaying && !ik.initiated) return;
            
            foreach (Transform bone in ik.bones)
            {
                if (bone == null) return;
            }

            Handles.color = color;
            GUI.color = color;

            // Display the bones
            for (int i = 0; i < ik.bones.Length; i++)
            {
                if (i < ik.bones.Length - 1) Handles.DrawLine(ik.bones[i].position, ik.bones[i + 1].position);
                Inspector.SphereCap(0, ik.bones[i].position, Quaternion.identity, IKSolverInspector.GetHandleSize(ik.bones[i].position));
            }

            // Selecting joint and manipulating IKPosition
            if (Application.isPlaying && ik.weight > 0)
            {
                if (ik.target != null)
                {
                    if (Inspector.SphereButton(ik.target.position, Quaternion.identity, IKSolverInspector.GetHandleSize(ik.target.position), IKSolverInspector.GetHandleSize(ik.target.position)))
                    {
                        Selection.activeGameObject = ik.target.gameObject;
                    }

                    Handles.DrawLine(ik.bones[ik.bones.Length - 1].transform.position, ik.target.position);
                }

                // Draw a transparent line from last bone to IKPosition
                Handles.color = new Color(color.r, color.g, color.b, color.a * ik.weight);
            }

            Handles.color = Color.white;
            GUI.color = Color.white;
        }
    }
}
