using UnityEngine;
using System.Collections;

namespace RootMotion.FinalIK {
	
	// TODO [HelpURL("http://www.root-motion.com/finalikdox/html/page9.html")]
	[AddComponentMenu("Scripts/RootMotion.FinalIK/IK/Leg IK")]
	public class LegIK : IK {

		// Open the User Manual URL
		[ContextMenu("User Manual")]
		protected override void OpenUserManual() {
			Debug.Log ("No User Manual page for this component yet, sorry."); // TODO
			//Application.OpenURL("http://www.root-motion.com/finalikdox/html/page9.html");
		}
		
		// Open the Script Reference URL
		[ContextMenu("Scrpt Reference")]
		protected override void OpenScriptReference() {
			Debug.Log ("No Script Reference for this component yet, sorry."); // TODO
			//Application.OpenURL("http://www.root-motion.com/finalikdox/html/class_root_motion_1_1_final_i_k_1_1_trigonometric_i_k.html");
		}
		
		// Link to the Final IK Google Group
		[ContextMenu("Support Group")]
		void SupportGroup() {
			Application.OpenURL("https://groups.google.com/forum/#!forum/final-ik");
		}
		
		// Link to the Final IK Asset Store thread in the Unity Community
		[ContextMenu("Asset Store Thread")]
		void ASThread() {
			Application.OpenURL("http://forum.unity3d.com/threads/final-ik-full-body-ik-aim-look-at-fabrik-ccd-ik-1-0-released.222685/");
		}

		/// <summary>
		/// The Leg %IK solver.
		/// </summary>
		public IKSolverLeg solver = new IKSolverLeg();
		
		public override IKSolver GetIKSolver() {
			return solver as IKSolver;
		}
	}
}
