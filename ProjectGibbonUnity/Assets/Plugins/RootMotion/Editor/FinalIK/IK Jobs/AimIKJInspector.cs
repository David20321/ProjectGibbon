using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace RootMotion.FinalIK
{
    [CustomEditor(typeof(AimIKJ))]
    public class AimIKJInspector : Editor
    {
        private AimIKJ script { get { return target as AimIKJ; } }

        void OnSceneGUI()
        {
            // Draw the scene veiw helpers
            AddScene(script, new Color(1f, 0f, 0.5f, 1f), true);
        }

        //Draws the scene view helpers for AimIKJ
		public static void AddScene(AimIKJ ik, Color color, bool modifiable)
        {
            // Protect from null reference errors
            if (ik.aimTransform == null) return;
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
                Inspector.SphereCap(0, ik.bones[i].transform.position, Quaternion.identity, IKSolverInspector.GetHandleSize(ik.bones[i].position));
            }

            if (ik.axis != Vector3.zero) Inspector.ConeCap(0, ik.aimTransform.position, Quaternion.LookRotation(ik.aimTransform.rotation * ik.axis), IKSolverInspector.GetHandleSize(ik.aimTransform.position) * 2f);

            // Selecting joint and manipulating IKPosition
            if (Application.isPlaying && ik.weight > 0)
            {
                if (ik.target != null)
                {
                   // Inspector.SphereCap(0, ik.target.position, Quaternion.identity, IKSolverInspector.GetHandleSize(ik.target.position));
                    Handles.DrawLine(ik.aimTransform.position, ik.target.position);

                    if (Inspector.SphereButton(ik.target.position, Quaternion.identity, IKSolverInspector.GetHandleSize(ik.target.position), IKSolverInspector.GetHandleSize(ik.target.position)))
                    {
                        Selection.activeGameObject = ik.target.gameObject;
                    }
                }
                
                // Draw a transparent line from transform to IKPosition
                Handles.color = new Color(color.r, color.g, color.b, color.a * ik.weight);
                Handles.DrawLine(ik.bones[ik.bones.Length - 1].position, ik.aimTransform.position);
            }

            Handles.color = color;

            // Pole
            if (Application.isPlaying && ik.poleWeight > 0f)
            {
                if (ik.poleTarget != null)
                {
                    Inspector.SphereCap(0, ik.poleTarget.position, Quaternion.identity, IKSolverInspector.GetHandleSize(ik.poleTarget.position) * 0.5f);
                    Handles.DrawLine(ik.aimTransform.position, ik.poleTarget.position);
                }
                    
                // Draw a transparent line from transform to polePosition
                Handles.color = new Color(color.r, color.g, color.b, color.a * ik.poleWeight);
                
            }

            Handles.color = Color.white;
            GUI.color = Color.white;
        }

    }
}
