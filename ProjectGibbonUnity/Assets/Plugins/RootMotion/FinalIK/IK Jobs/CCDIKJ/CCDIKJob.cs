using System.Collections;
using UnityEngine;
using Unity.Collections;
using UnityEngine.Experimental.Animations;

namespace RootMotion.FinalIK
{

    /// <summary>
    /// CCDIK AnimationJob.
    /// </summary>
    public struct CCDIKJob : IAnimationJob
    {

        public TransformSceneHandle _target;
        public PropertySceneHandle _IKPositionWeight;
        public PropertySceneHandle _maxIterations;
        public PropertySceneHandle _tolerance;
        public PropertySceneHandle _XY;
        public PropertySceneHandle _useRotationLimits;

        private NativeArray<TransformStreamHandle> bones;
        private NativeArray<PropertySceneHandle> boneWeights;
        private NativeArray<float> boneSqrMags;
        private float chainSqrMag;
        private Vector3 lastLocalDirection;

        public void Setup(Animator animator, Transform[] bones, Transform target)
        {
            this.bones = new NativeArray<TransformStreamHandle>(bones.Length, Allocator.Persistent);
            this.boneWeights = new NativeArray<PropertySceneHandle>(bones.Length - 1, Allocator.Persistent);
            this.boneSqrMags = new NativeArray<float>(bones.Length - 1, Allocator.Persistent);

            for (int i = 0; i < this.bones.Length; i++)
            {
                this.bones[i] = animator.BindStreamTransform(bones[i]);
            }

            for (int i = 0; i < this.bones.Length - 1; i++)
            {
                var boneParams = bones[i].gameObject.GetComponent<IKJBoneParams>();
                if (boneParams == null) boneParams = bones[i].gameObject.AddComponent<IKJBoneParams>();

                this.boneWeights[i] = animator.BindSceneProperty(bones[i].transform, typeof(IKJBoneParams), "weight");
            }

            // Rotation Limits
            SetUpRotationLimits(animator, bones);

            _target = animator.BindSceneTransform(target);
            _IKPositionWeight = animator.BindSceneProperty(animator.transform, typeof(CCDIKJ), "weight");
            _maxIterations = animator.BindSceneProperty(animator.transform, typeof(CCDIKJ), "maxIterations");
            _tolerance = animator.BindSceneProperty(animator.transform, typeof(CCDIKJ), "tolerance");
            _XY = animator.BindSceneProperty(animator.transform, typeof(CCDIKJ), "XY");
            _useRotationLimits = animator.BindSceneProperty(animator.transform, typeof(CCDIKJ), "useRotationLimits");
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

            float w = _IKPositionWeight.GetFloat(s);

            if (w <= 0) return;
            w = Mathf.Min(w, 1f);

            Read(s);

            bool XY = _XY.GetBool(s);
            float maxIterations = _maxIterations.GetInt(s);
            float tolerance = _tolerance.GetFloat(s);
            bool useRotationLimits = _useRotationLimits.GetBool(s);

            Vector3 IKPosition = _target.GetPosition(s);
            if (XY) IKPosition.z = bones[0].GetPosition(s).z;

            Vector3 singularityOffset = maxIterations > 1 ? GetSingularityOffset(s, IKPosition, useRotationLimits) : Vector3.zero;

            // Iterating the solver
            int it = 1;
            
            for (int i = 0; i < maxIterations; i++)
            {
                // Optimizations
                Vector3 localDirection = GetLocalDirection(s);
                if (singularityOffset == Vector3.zero && i >= 1 && tolerance > 0 && GetPositionOffset(s, localDirection) < tolerance * tolerance) break;
                lastLocalDirection = localDirection;

                Solve(s, IKPosition + (i == 0 ? singularityOffset : Vector3.zero), XY, w, it, useRotationLimits);

                it++;
                if (it >= bones.Length - 1) it -= bones.Length - 2;
            }

            lastLocalDirection = GetLocalDirection(s);
        }

        private void Read(AnimationStream s)
        {
            chainSqrMag = 0;

            for (int i = 0; i < bones.Length; i++)
            {
                // Calculate bone and chain sqr magnitudes
                if (i < bones.Length - 1)
                {
                    boneSqrMags[i] = (bones[i].GetPosition(s) - bones[i + 1].GetPosition(s)).sqrMagnitude;
                    chainSqrMag += boneSqrMags[i];
                }
            }
        }

        private void Solve(AnimationStream s, Vector3 targetPosition, bool XY, float weight, int it, bool useRotationLimits)
        {
            for (int i = bones.Length - 2; i > -1; i--)
            {
                //CCD tends to overemphasise the rotations of the bones closer to the target position. Reducing bone weight down the hierarchy will compensate for this effect.
                float w = weight * boneWeights[i].GetFloat(s);

                if (w > 0f)
                {
                    Vector3 bonePos = bones[i].GetPosition(s);
                    Vector3 toLastBone = bones[bones.Length - 1].GetPosition(s) - bonePos;
                    Vector3 toTarget = targetPosition - bonePos;

                    // Get the rotation to direct the last bone to the target
                    if (XY)
                    {
                        float angleToLastBone = Mathf.Atan2(toLastBone.x, toLastBone.y) * Mathf.Rad2Deg;
                        float angleToTarget = Mathf.Atan2(toTarget.x, toTarget.y) * Mathf.Rad2Deg;

                        // Rotation to direct the last bone to the target
                        bones[i].SetRotation(s, Quaternion.AngleAxis(Mathf.DeltaAngle(angleToLastBone, angleToTarget) * w, Vector3.back) * bones[i].GetRotation(s));
                    }
                    else
                    {
                        Quaternion boneRot = bones[i].GetRotation(s);
                        Quaternion targetRotation = Quaternion.FromToRotation(toLastBone, toTarget) * boneRot;

                        if (w >= 1) bones[i].SetRotation(s, targetRotation);
                        else bones[i].SetRotation(s, Quaternion.Lerp(boneRot, targetRotation, w));
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
        }

        //Gets the direction from last bone to first bone in first bone's local space.
        private Vector3 GetLocalDirection(AnimationStream s)
        {
            return Quaternion.Inverse(bones[0].GetRotation(s)) * (bones[bones.Length - 1].GetPosition(s) - bones[0].GetPosition(s));
        }

        //Gets the offset from last position of the last bone to its current position.
        private float GetPositionOffset(AnimationStream s, Vector3 localDirection)
        {
            return Vector3.SqrMagnitude(localDirection - lastLocalDirection);
        }

        // Get target offset to break out of the linear singularity issue
        private Vector3 GetSingularityOffset(AnimationStream s, Vector3 IKPosition, bool useRotationLimits)
        {
            if (!SingularityDetected(s, IKPosition)) return Vector3.zero;

            Vector3 IKDirection = (IKPosition - bones[0].GetPosition(s)).normalized;

            Vector3 secondaryDirection = new Vector3(IKDirection.y, IKDirection.z, IKDirection.x);

            // Avoiding getting locked by the Hinge Rotation Limit
            if (useRotationLimits && hingeFlags[bones.Length - 2] == 1)
            {
                secondaryDirection = bones[bones.Length - 2].GetRotation(s) * limitAxisArray[bones.Length - 2];
            }

            return Vector3.Cross(IKDirection, secondaryDirection) * Mathf.Sqrt(boneSqrMags[bones.Length - 2]) * 0.5f;
        }

        // Detects linear singularity issue when the direction from first bone to IKPosition matches the direction from first bone to the last bone.
        private bool SingularityDetected(AnimationStream s, Vector3 IKPosition)
        {
            Vector3 firstBonePos = bones[0].GetPosition(s);
            Vector3 toLastBone = bones[bones.Length - 1].GetPosition(s) - firstBonePos;
            Vector3 toIKPosition = IKPosition - firstBonePos;

            float toLastBoneDistance = toLastBone.sqrMagnitude;
            float toIKPositionDistance = toIKPosition.sqrMagnitude;

            if (toLastBoneDistance < toIKPositionDistance) return false;
            if (toLastBoneDistance < chainSqrMag - (boneSqrMags[bones.Length - 2] * 0.1f)) return false;
            if (toLastBoneDistance == 0) return false;
            if (toIKPositionDistance == 0) return false;

            float dot = Vector3.Dot(toLastBone, toIKPosition);
            if (dot < 0.999f) return false;

            return true;
        }

        public void Dispose()
        {
            bones.Dispose();
            boneWeights.Dispose();
            boneSqrMags.Dispose();

            DisposeRotationLimits();
        }
    }
}
