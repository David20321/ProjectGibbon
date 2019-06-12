using System.Collections;
using UnityEngine;
using UnityEngine.Playables;
using UnityEngine.Animations;
using UnityEngine.Experimental.Animations;

namespace RootMotion.FinalIK
{

    /// <summary>
    /// /// AnimationJob version of the Aim %IK solver.
    /// </summary>
    [RequireComponent(typeof(Animator))]
    [AddComponentMenu("Scripts/RootMotion.FinalIK/IK Jobs/Aim IKJ")]
    public class AimIKJ : MonoBehaviour
    {
        [Tooltip("The target Transform of this solver.")]
        public Transform target;

        [Tooltip("Optional secondary target for another axis of the 'Aim Transform'. Needs 'Pole Weight' to be greater than 0 to have any effect.")]
        public Transform poleTarget;

        [Tooltip("The transform that you want to be aimed at the target. Needs to be a lineal descendant of the bone hierarchy. For example, if you wish to aim a gun, it should be the gun, one of it's children or the hand bone.")]
        public Transform aimTransform;

        [Space(5)]

        [Tooltip("The X value of the local axis of the 'Aim Transform' that you want to be aimed at IKPosition.")]
        public float axisX;

        [Tooltip("The Y value of the local axis of the 'Aim Transform' that you want to be aimed at IKPosition.")]
        public float axisY;

        [Tooltip("The Z value of the local axis of the 'Aim Transform' that you want to be aimed at IKPosition.")]
        public float axisZ = 1f;

        [Space(5)]

        [Tooltip("The X value of the local axis of the 'Aim Transform' that you want oriented towards the 'Pole Target'.")]
        public float poleAxisX;

        [Tooltip("The Y value of the local axis of the 'Aim Transform' that you want oriented towards the 'Pole Target'.")]
        public float poleAxisY = 1f;

        [Tooltip("The Z value of the local axis of the 'Aim Transform' that you want oriented towards the 'Pole Target'.")]
        public float poleAxisZ;

        [Space(5)]

        [Tooltip("The master weight of this solver.")]
        [Range(0f, 1f)] public float weight = 1f;

        [Tooltip("The weight of the 'Pole Target'")]
        [Range(0f, 1f)] public float poleWeight;
        
        [Tooltip("Minimum angular offset from last reached angle. Will stop solving if offset is less than tolerance.If tolerance is zero, will iterate until maxIterations.")]
        public float tolerance;

        [Tooltip("Max solver iterations per frame. If target position offset is less than 'Tolerance', will stop solving.")]
        public int maxIterations = 4;

        [Tooltip("Clamping rotation of the solver. 0 is free rotation, 1 is completely clamped to animated rotation.")]
        [Range(0f, 1f)] public float clampWeight = 0.1f;

        [Tooltip("Number of sine smoothing iterations applied on clamping to make the clamping point smoother.")]
        [Range(0, 2)] public int clampSmoothing = 2;

        [Tooltip("If true, rotation limits (if existing) will be applied on each iteration. Only RotationLimitAngle and RotationLimitHinge can be used with this solver.")]
        public bool useRotationLimits = true;

        [Tooltip("Useful for 2D games. If true, will solve only in the XY plane.")]
        public bool XY;

        [Space(5)]

        [Tooltip("The list of bones used by the solver. Must be assigned in order of hierarchy. All bones must be in the same hierarchy branch.")]
        public Transform[] bones = new Transform[0];
       
        /// <summary>
        /// Returns true if successfully initiated.
        /// </summary>
        public bool initiated { get; private set; }

        private Animator animator;
        private PlayableGraph graph;
        private AnimationScriptPlayable IKPlayable;
        private AimIKJob job;

        public Vector3 axis
        {
            get
            {
                return new Vector3(axisX, axisY, axisZ);
            }
            set
            {
                axisX = value.x;
                axisY = value.y;
                axisZ = value.z;
            }
        }

        public Vector3 poleAxis
        {
            get
            {
                return new Vector3(poleAxisX, poleAxisY, poleAxisZ);
            }
            set
            {
                poleAxisX = value.x;
                poleAxisY = value.y;
                poleAxisZ = value.z;
            }
        }

        void OnEnable()
        {
            // Validation
            if (bones.Length < 1) return;
            if (axis == Vector3.zero) return;
            if (poleAxis == Vector3.zero && poleWeight > 0f) return;

            animator = GetComponent<Animator>();

            // Creating the target
            if (aimTransform == null) aimTransform = bones[bones.Length - 1];

            if (target == null)
            {
                target = new GameObject("AimIKJ Target (" + name + ")").transform;
                target.position = bones[bones.Length - 1].position + aimTransform.rotation * axis * 3f;
            }

            if (poleTarget == null)
            {
                poleTarget = new GameObject("AimIKJ Pole Target (" + name + ")").transform;
                poleTarget.position = bones[bones.Length - 1].position + aimTransform.rotation * poleAxis * 3f;
            }
            
            // Creating the grapsh and output
            graph = PlayableGraph.Create("AimIKJ");
            var output = AnimationPlayableOutput.Create(graph, "ouput", GetComponent<Animator>());

            // Setting up the Job
            job = new AimIKJob();
            job.Setup(GetComponent<Animator>(), bones, target, poleTarget, aimTransform);

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
                if (bones.Length < 1)
                {
                    Debug.LogError("AimIKJ needs at least 1 bone to run.", transform);
                    enabled = false;
                    return;
                }

                if (new Vector3(axisX, axisY, axisZ) == Vector3.zero)
                {
                    Debug.Log("AimIKJ 'Axis' must not be zero.", transform);
                    return;
                }
                if (new Vector3(poleAxisX, poleAxisY, poleAxisZ) == Vector3.zero && poleWeight > 0f)
                {
                    Debug.Log("AimIKJ 'Pole Axis' must not be zero when 'Pole Weight' is greater than 0.", transform);
                    return;
                }

                OnEnable();
                return;
            }

            // This solver is useless without its targets
            if (initiated)
            {
                if (target == null)
                {
                    Debug.LogError("AimIKJ 'Target' has gone missing, destroying AimIKJ.", transform);
                    Destroy(this);
                    return;
                }

                if (poleTarget == null)
                {
                    Debug.LogError("AimIKJ 'Pole Target' has gone missing, destroying AimIKJ.", transform);
                    Destroy(this);
                    return;
                }

                if (aimTransform == null)
                {
                    Debug.LogError("AimIKJ 'Aim transform' has gone missing, destroying AimIKJ.", transform);
                    Destroy(this);
                    return;
                }
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
