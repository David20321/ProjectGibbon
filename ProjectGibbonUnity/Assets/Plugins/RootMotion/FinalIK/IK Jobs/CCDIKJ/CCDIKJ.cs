using System.Collections;
using UnityEngine;
using UnityEngine.Playables;
using UnityEngine.Animations;
using UnityEngine.Experimental.Animations;

namespace RootMotion.FinalIK
{

    /// <summary>
    /// /// AnimationJob version of the CCD (Cyclic Coordinate Descent) %IK solver.
    /// </summary>
    [RequireComponent(typeof(Animator))]
    [AddComponentMenu("Scripts/RootMotion.FinalIK/IK Jobs/CCD IKJ")]
    public class CCDIKJ : MonoBehaviour
    {
        [Tooltip("The target Transform of this solver.")]
        public Transform target;

        [Tooltip("The master weight of this solver.")]
        [Range(0f, 1f)] public float weight = 1f;

        [Tooltip("Minimum offset from last reached position. Will stop solving if offset is less than tolerance.If tolerance is zero, will iterate until maxIterations.")]
        public float tolerance;

        [Tooltip("Max solver iterations per frame. If target position offset is less than 'Tolerance', will stop solving.")]
        public int maxIterations = 4;

        [Tooltip("If true, rotation limits (if existing) will be applied on each iteration. Only RotationLimitAngle and RotationLimitHinge can be used with this solver.")]
        public bool useRotationLimits = true;

        [Tooltip("Useful for 2D games. If true, will solve only in the XY plane.")]
        public bool XY;

        [Tooltip("The list of bones used by the solver. Must be assigned in order of hierarchy. All bones must be in the same hierarchy branch.")]
        public Transform[] bones = new Transform[0];
        
        /// <summary>
        /// Returns true if successfully initiated.
        /// </summary>
        public bool initiated { get; private set; }

        private Animator animator;
        private PlayableGraph graph;
        private AnimationScriptPlayable IKPlayable;
        private CCDIKJob job;
        
        void OnEnable()
        {
            // Validation
            if (bones.Length < 3) return;
            
            animator = GetComponent<Animator>();

            // Creating the target
            if (target == null)
            {
                target = new GameObject("CCDIKJ Target (" + name + ")").transform;
                target.position = bones[bones.Length - 1].position;
                target.rotation = bones[bones.Length - 1].rotation;
            }

            // Creating the grapsh and output
            graph = PlayableGraph.Create("CCDIKJ");
            var output = AnimationPlayableOutput.Create(graph, "ouput", GetComponent<Animator>());

            // Setting up the Job
            job = new CCDIKJob();
            job.Setup(GetComponent<Animator>(), bones, target);

            // Creating the playable
            IKPlayable = AnimationScriptPlayable.Create(graph, job);

            var controllerPlayable = AnimatorControllerPlayable.Create(graph, animator.runtimeAnimatorController);

            IKPlayable.AddInput(controllerPlayable, 0, 1.0f);

            // Starting the graphs
            output.SetSourcePlayable(IKPlayable);
            graph.Play();

            initiated = true;
        }

        void Update()
        {
            // Enable adding this component in runtime
            if (!initiated)
            {
                if (bones.Length < 3)
                {
                    Debug.LogError("CCDIKJ needs at least 3 bones to run.", transform);
                    enabled = false;
                    return;
                }

                OnEnable();
                return;
            }

            // This solver is useless without a target
            if (initiated && target == null)
            {
                Debug.LogError("CCDIKJ 'Target' has gone missing. Destroying CCDIKJ.", transform);
                Destroy(this);
            }
        }

        // Disposing of memory
        void OnDisable()
        {
            if (!initiated) return;

            job.Dispose();
            graph.Destroy();
            Object.Destroy(target);
        }
    }
}
