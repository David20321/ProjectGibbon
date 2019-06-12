using System.Collections;
using UnityEngine;
using Unity.Collections;
using UnityEngine.Experimental.Animations;

// TODO Icons

namespace RootMotion.FinalIK
{

    /// <summary>
    /// AimIK AnimationJob.
    /// </summary>
    public struct AimIKJob : IAnimationJob
    {

        public TransformSceneHandle _target;
        public TransformSceneHandle _poleTarget;
        public TransformStreamHandle _transform;
        public PropertySceneHandle _IKPositionWeight;
        public PropertySceneHandle _poleWeight;
        public PropertySceneHandle _axisX;
        public PropertySceneHandle _axisY;
        public PropertySceneHandle _axisZ;
        public PropertySceneHandle _poleAxisX;
        public PropertySceneHandle _poleAxisY;
        public PropertySceneHandle _poleAxisZ;
        public PropertySceneHandle _clampWeight;
        public PropertySceneHandle _clampSmoothing;
        public PropertySceneHandle _maxIterations;
        public PropertySceneHandle _tolerance;
        public PropertySceneHandle _XY;
        public PropertySceneHandle _useRotationLimits;

        private NativeArray<TransformStreamHandle> bones;
        private NativeArray<PropertySceneHandle> boneWeights;
        private Vector3 lastLocalDirection;
        private float step;

        public void Setup(Animator animator, Transform[] bones, Transform target, Transform poleTarget, Transform aimTransform)
        {
            this.bones = new NativeArray<TransformStreamHandle>(bones.Length, Allocator.Persistent);
            this.boneWeights = new NativeArray<PropertySceneHandle>(bones.Length, Allocator.Persistent);
            
            for (int i = 0; i < this.bones.Length; i++)
            {
                this.bones[i] = animator.BindStreamTransform(bones[i]);
            }

            
            for (int i = 0; i < this.bones.Length; i++)
            {
                var boneParams = bones[i].gameObject.GetComponent<IKJBoneParams>();
                if (boneParams == null) boneParams = bones[i].gameObject.AddComponent<IKJBoneParams>();

                this.boneWeights[i] = animator.BindSceneProperty(bones[i].transform, typeof(IKJBoneParams), "weight");
            }

            // Rotation Limits
            SetUpRotationLimits(animator, bones);

            _target = animator.BindSceneTransform(target);
            _poleTarget = animator.BindSceneTransform(poleTarget);
            _transform = animator.BindStreamTransform(aimTransform);
            _IKPositionWeight = animator.BindSceneProperty(animator.transform, typeof(AimIKJ), "weight");
            _poleWeight = animator.BindSceneProperty(animator.transform, typeof(AimIKJ), "poleWeight");
            _axisX = animator.BindSceneProperty(animator.transform, typeof(AimIKJ), "axisX");
            _axisY = animator.BindSceneProperty(animator.transform, typeof(AimIKJ), "axisY");
            _axisZ = animator.BindSceneProperty(animator.transform, typeof(AimIKJ), "axisZ");
            _poleAxisX = animator.BindSceneProperty(animator.transform, typeof(AimIKJ), "poleAxisX");
            _poleAxisY = animator.BindSceneProperty(animator.transform, typeof(AimIKJ), "poleAxisY");
            _poleAxisZ = animator.BindSceneProperty(animator.transform, typeof(AimIKJ), "poleAxisZ");
            _clampWeight = animator.BindSceneProperty(animator.transform, typeof(AimIKJ), "clampWeight");
            _clampSmoothing = animator.BindSceneProperty(animator.transform, typeof(AimIKJ), "clampSmoothing");
            _maxIterations = animator.BindSceneProperty(animator.transform, typeof(AimIKJ), "maxIterations");
            _tolerance = animator.BindSceneProperty(animator.transform, typeof(AimIKJ), "tolerance");
            _XY = animator.BindSceneProperty(animator.transform, typeof(AimIKJ), "XY");
            _useRotationLimits = animator.BindSceneProperty(animator.transform, typeof(AimIKJ), "useRotationLimits");

            step = 1f / (float)bones.Length;
        }

        #region Rotation Limits

        // All limits
        private NativeArray<Quaternion> limitDefaultLocalRotationArray;
        private NativeArray<Vector3> limitAxisArray;

        // Hinge
        private NativeArray<int> hingeFlags;
        private NativeArray<PropertySceneHandle> hingeMinArray;
        private NativeArray<PropertySceneHandle> hingeMaxArray;
        private NativeArray<PropertySceneHandle> hingeUseLimitsArray;
        private NativeArray<Quaternion> hingeLastRotations;
        private NativeArray<float> hingeLastAngles;

        // Angle
        private NativeArray<int> angleFlags;
        private NativeArray<Vector3> angleSecondaryAxisArray;
        private NativeArray<PropertySceneHandle> angleLimitArray;
        private NativeArray<PropertySceneHandle> angleTwistLimitArray;

        private void SetUpRotationLimits(Animator animator, Transform[] bones)
        {
            // All limits
            this.limitDefaultLocalRotationArray = new NativeArray<Quaternion>(bones.Length, Allocator.Persistent);
            this.limitAxisArray = new NativeArray<Vector3>(bones.Length, Allocator.Persistent);

            // Hinge
            this.hingeFlags = new NativeArray<int>(bones.Length, Allocator.Persistent);
            this.hingeMinArray = new NativeArray<PropertySceneHandle>(bones.Length, Allocator.Persistent);
            this.hingeMaxArray = new NativeArray<PropertySceneHandle>(bones.Length, Allocator.Persistent);
            this.hingeUseLimitsArray = new NativeArray<PropertySceneHandle>(bones.Length, Allocator.Persistent);
            this.hingeLastRotations = new NativeArray<Quaternion>(bones.Length, Allocator.Persistent);
            this.hingeLastAngles = new NativeArray<float>(bones.Length, Allocator.Persistent);

            // Angle
            this.angleFlags = new NativeArray<int>(bones.Length, Allocator.Persistent);
            this.angleSecondaryAxisArray = new NativeArray<Vector3>(bones.Length, Allocator.Persistent);
            this.angleLimitArray = new NativeArray<PropertySceneHandle>(bones.Length, Allocator.Persistent);
            this.angleTwistLimitArray = new NativeArray<PropertySceneHandle>(bones.Length, Allocator.Persistent);

            for (int i = 0; i < bones.Length - 1; i++)
            {
                this.hingeFlags[i] = 0;
                this.angleFlags[i] = 0;

                var limit = bones[i].GetComponent<RotationLimit>();
                if (limit != null)
                {
                    // All limits
                    this.limitDefaultLocalRotationArray[i] = bones[i].localRotation;
                    this.limitAxisArray[i] = limit.axis;

                    limit.Disable();

                    // Hinge
                    if (limit is RotationLimitHinge)
                    {
                        //var hinge = limit as RotationLimitHinge;

                        this.hingeFlags[i] = 1;
                        this.hingeMinArray[i] = animator.BindSceneProperty(bones[i].transform, typeof(RotationLimitHinge), "min");
                        this.hingeMaxArray[i] = animator.BindSceneProperty(bones[i].transform, typeof(RotationLimitHinge), "max");
                        this.hingeUseLimitsArray[i] = animator.BindSceneProperty(bones[i].transform, typeof(RotationLimitHinge), "useLimits");
                        this.hingeLastRotations[i] = bones[i].localRotation;
                        this.hingeLastAngles[i] = 0f;
                    }

                    // Angle
                    if (limit is RotationLimitAngle)
                    {
                        var angle = limit as RotationLimitAngle;

                        this.angleFlags[i] = 1;
                        this.angleSecondaryAxisArray[i] = angle.secondaryAxis;
                        this.angleLimitArray[i] = animator.BindSceneProperty(bones[i].transform, typeof(RotationLimitAngle), "limit");
                        this.angleTwistLimitArray[i] = animator.BindSceneProperty(bones[i].transform, typeof(RotationLimitAngle), "twistLimit");

                    }
                }
            }
        }

        private void DisposeRotationLimits()
        {
            // All limits
            limitDefaultLocalRotationArray.Dispose();
            limitAxisArray.Dispose();

            // Hinge
            hingeFlags.Dispose();
            hingeMinArray.Dispose();
            hingeMaxArray.Dispose();
            hingeUseLimitsArray.Dispose();
            hingeLastRotations.Dispose();
            hingeLastAngles.Dispose();

            // Angle
            angleFlags.Dispose();
            angleSecondaryAxisArray.Dispose();
            angleLimitArray.Dispose();
            angleTwistLimitArray.Dispose();
        }

        #endregion Rotation Limits

        public void ProcessRootMotion(AnimationStream stream)
        {
        }

        public void ProcessAnimation(AnimationStream stream)
        {
            Update(stream);
        }

        private void Update(AnimationStream s)
        {
            if (!_target.IsValid(s)) return;
            if (!_poleTarget.IsValid(s)) return;
            if (!_transform.IsValid(s)) return;

            Vector3 axis = new Vector3(_axisX.GetFloat(s), _axisY.GetFloat(s), _axisZ.GetFloat(s));
            Vector3 poleAxis = new Vector3(_poleAxisX.GetFloat(s), _poleAxisY.GetFloat(s), _poleAxisZ.GetFloat(s));

            float poleWeight = _poleWeight.GetFloat(s);
            poleWeight = Mathf.Clamp(poleWeight, 0f, 1f);

            if (axis == Vector3.zero) return;
            if (poleAxis == Vector3.zero && poleWeight > 0f) return;

            float IKPositionWeight = _IKPositionWeight.GetFloat(s);

            if (IKPositionWeight <= 0) return;
            IKPositionWeight = Mathf.Min(IKPositionWeight, 1f);
           
            bool XY = _XY.GetBool(s);
            float maxIterations = _maxIterations.GetInt(s);
            float tolerance = _tolerance.GetFloat(s);
            bool useRotationLimits = _useRotationLimits.GetBool(s);

            Vector3 IKPosition = _target.GetPosition(s);
            if (XY) IKPosition.z = bones[0].GetPosition(s).z;

            Vector3 polePosition = _poleTarget.GetPosition(s);
            if (XY) polePosition.z = IKPosition.z;            

            float clampWeight = _clampWeight.GetFloat(s);
            clampWeight = Mathf.Clamp(clampWeight, 0f, 1f);
            int clampSmoothing = _clampSmoothing.GetInt(s);
            clampSmoothing = Mathf.Clamp(clampSmoothing, 0, 2);

            Vector3 transformPosition = _transform.GetPosition(s);
            Quaternion transformRotation = _transform.GetRotation(s);
            Vector3 transformAxis = transformRotation * axis;

            Vector3 clampedIKPosition = GetClampedIKPosition(s, clampWeight, clampSmoothing, IKPosition, transformPosition, transformAxis);

            Vector3 dir = clampedIKPosition - transformPosition;
            dir = Vector3.Slerp(transformAxis * dir.magnitude, dir, IKPositionWeight);
            clampedIKPosition = transformPosition + dir;

            // Iterating the solver
            for (int i = 0; i < maxIterations; i++)
            {
                // Optimizations
                if (i >= 0 && tolerance > 0 && GetAngle(s, axis, IKPosition) < tolerance) break;
                lastLocalDirection = GetLocalDirection(s, _transform.GetRotation(s) * axis);

                for (int n = 0; n < bones.Length - 1; n++)
                {
                    RotateToTarget(s, clampedIKPosition, polePosition, n, step * (n + 1) * IKPositionWeight * boneWeights[n].GetFloat(s), poleWeight, XY, useRotationLimits, axis, poleAxis);
                }

                RotateToTarget(s, clampedIKPosition, polePosition, bones.Length - 1, IKPositionWeight * boneWeights[bones.Length - 1].GetFloat(s), poleWeight, XY, useRotationLimits, axis, poleAxis);
            }

            lastLocalDirection = GetLocalDirection(s, _transform.GetRotation(s) * axis);
        }

        // Rotating bone to get transform aim closer to target
        private void RotateToTarget(AnimationStream s, Vector3 targetPosition, Vector3 polePosition, int i, float weight, float poleWeight, bool XY, bool useRotationLimits, Vector3 axis, Vector3 poleAxis)
        {
            // Swing
            if (XY)
            {
                if (weight >= 0f)
                {
                    Vector3 dir = _transform.GetRotation(s) * axis;
                    Vector3 targetDir = targetPosition - _transform.GetPosition(s);

                    float angleDir = Mathf.Atan2(dir.x, dir.y) * Mathf.Rad2Deg;
                    float angleTarget = Mathf.Atan2(targetDir.x, targetDir.y) * Mathf.Rad2Deg;

                    bones[i].SetRotation(s, Quaternion.AngleAxis(Mathf.DeltaAngle(angleDir, angleTarget), Vector3.back) * bones[i].GetRotation(s));
                }
            }
            else
            {
                if (weight >= 0f)
                {
                    Quaternion rotationOffset = Quaternion.FromToRotation(_transform.GetRotation(s) * axis, targetPosition - _transform.GetPosition(s));

                    if (weight >= 1f)
                    {
                        bones[i].SetRotation(s, rotationOffset * bones[i].GetRotation(s));
                    }
                    else
                    {
                        bones[i].SetRotation(s, Quaternion.Lerp(Quaternion.identity, rotationOffset, weight) * bones[i].GetRotation(s));
                    }
                }

                // Pole
                if (poleWeight > 0f)
                {
                    Vector3 poleDirection = polePosition - _transform.GetPosition(s);

                    // Ortho-normalize to transform axis to make this a twisting only operation
                    Vector3 poleDirOrtho = poleDirection;
                    Vector3 normal = _transform.GetRotation(s) * axis;
                    Vector3.OrthoNormalize(ref normal, ref poleDirOrtho);

                    Quaternion toPole = Quaternion.FromToRotation(_transform.GetRotation(s) * poleAxis, poleDirOrtho);
                    bones[i].SetRotation(s, Quaternion.Lerp(Quaternion.identity, toPole, weight * poleWeight) * bones[i].GetRotation(s));
                }
            }

            // Rotation Constraints
            if (useRotationLimits)
            {
                if (hingeFlags[i] == 1)
                {
                    Quaternion localRotation = Quaternion.Inverse(limitDefaultLocalRotationArray[i]) * bones[i].GetLocalRotation(s);
                    Quaternion lastRotation = hingeLastRotations[i];
                    float lastAngle = hingeLastAngles[i];
                    Quaternion r = RotationLimitUtilities.LimitHinge(localRotation, hingeMinArray[i].GetFloat(s), hingeMaxArray[i].GetFloat(s), hingeUseLimitsArray[i].GetBool(s), limitAxisArray[i], ref lastRotation, ref lastAngle);
                    hingeLastRotations[i] = lastRotation;
                    hingeLastAngles[i] = lastAngle;
                    bones[i].SetLocalRotation(s, limitDefaultLocalRotationArray[i] * r);
                }
                else if (angleFlags[i] == 1)
                {
                    Quaternion localRotation = Quaternion.Inverse(limitDefaultLocalRotationArray[i]) * bones[i].GetLocalRotation(s);
                    Quaternion r = RotationLimitUtilities.LimitAngle(localRotation, limitAxisArray[i], angleSecondaryAxisArray[i], angleLimitArray[i].GetFloat(s), angleTwistLimitArray[i].GetFloat(s));
                    bones[i].SetLocalRotation(s, limitDefaultLocalRotationArray[i] * r);
                }
            }
        }


        public float GetAngle(AnimationStream s, Vector3 axis, Vector3 IKPosition)
        {
            return Vector3.Angle(_transform.GetRotation(s) * axis, IKPosition - _transform.GetPosition(s));
        }

        //Clamping the IKPosition to legal range
        private Vector3 GetClampedIKPosition(AnimationStream s, float clampWeight, int clampSmoothing, Vector3 IKPosition, Vector3 transformPosition, Vector3 transformAxis)
        {
            if (clampWeight <= 0f) return IKPosition;

            if (clampWeight >= 1f) return transformPosition + transformAxis * (IKPosition - transformPosition).magnitude;

            // Getting the dot product of IK direction and transformAxis
            float angle = Vector3.Angle(transformAxis, (IKPosition - transformPosition));
            float dot = 1f - (angle / 180f);

            // Clamping the target
            float targetClampMlp = clampWeight > 0 ? Mathf.Clamp(1f - ((clampWeight - dot) / (1f - dot)), 0f, 1f) : 1f;

            // Calculating the clamp multiplier
            float clampMlp = clampWeight > 0 ? Mathf.Clamp(dot / clampWeight, 0f, 1f) : 1f;

            for (int i = 0; i < clampSmoothing; i++)
            {
                float sinF = clampMlp * Mathf.PI * 0.5f;
                clampMlp = Mathf.Sin(sinF);
            }

            // Slerping the IK direction (don't use Lerp here, it breaks it)
            return transformPosition + Vector3.Slerp(transformAxis * 10f, IKPosition - transformPosition, clampMlp * targetClampMlp);
        }

        //Gets the direction from last bone to first bone in first bone's local space.
        private Vector3 GetLocalDirection(AnimationStream s, Vector3 transformAxis)
        {
            return Quaternion.Inverse(bones[0].GetRotation(s)) * transformAxis;
        }

        //Gets the offset from last position of the last bone to its current position.
        private float GetPositionOffset(AnimationStream s, Vector3 localDirection)
        {
            return Vector3.SqrMagnitude(localDirection - lastLocalDirection);
        }

        public void Dispose()
        {
            bones.Dispose();
            boneWeights.Dispose();
            
            DisposeRotationLimits();
        }
    }
}