using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RootMotion.FinalIK
{

    public static class RotationLimitUtilities
    {

        //Apply the hinge rotation limit
        public static Quaternion LimitHinge(Quaternion rotation, float min, float max, bool useLimits, Vector3 axis, ref Quaternion lastRotation, ref float lastAngle)
        {
            // If limit is zero return rotation fixed to axis
            if (min == 0 && max == 0 && useLimits) return Quaternion.AngleAxis(0, axis);

            // Get 1 degree of freedom rotation along axis
            Quaternion free1DOF = Limit1DOF(rotation, axis);
            if (!useLimits) return free1DOF;

            // Get offset from last rotation in angle-axis representation
            Quaternion addR = free1DOF * Quaternion.Inverse(lastRotation);

            float addA = Quaternion.Angle(Quaternion.identity, addR);

            Vector3 secondaryAxis = new Vector3(axis.z, axis.x, axis.y);
            Vector3 cross = Vector3.Cross(secondaryAxis, axis);
            if (Vector3.Dot(addR * secondaryAxis, cross) > 0f) addA = -addA;

            // Clamp to limits
            lastAngle = Mathf.Clamp(lastAngle + addA, min, max);
            lastRotation = Quaternion.AngleAxis(lastAngle, axis);
            return lastRotation;
        }

        // Limits rotation about a single axis
        public static Quaternion Limit1DOF(Quaternion rotation, Vector3 axis)
        {
            return Quaternion.FromToRotation(rotation * axis, axis) * rotation;
        }

        //Limits the rotation in the local space of this instance's Transform.
        public static Quaternion LimitAngle(Quaternion rotation, Vector3 axis, Vector3 secondaryAxis, float limit, float twistLimit)
        {
            // Subtracting off-limits swing
            Quaternion swing = LimitSwing(rotation, axis, limit);

            // Apply twist limits
            return LimitTwist(swing, axis, secondaryAxis, twistLimit);
        }

        // Apply angular swing limits
        public static Quaternion LimitSwing(Quaternion rotation, Vector3 axis, float limit)
        {
            if (axis == Vector3.zero) return rotation; // Ignore with zero axes
            if (rotation == Quaternion.identity) return rotation; // Assuming initial rotation is in the reachable area
            if (limit >= 180) return rotation;

            Vector3 swingAxis = rotation * axis;

            // Get the limited swing axis
            Quaternion swingRotation = Quaternion.FromToRotation(axis, swingAxis);
            Quaternion limitedSwingRotation = Quaternion.RotateTowards(Quaternion.identity, swingRotation, limit);

            // Rotation from current(illegal) swing rotation to the limited(legal) swing rotation
            Quaternion toLimits = Quaternion.FromToRotation(swingAxis, limitedSwingRotation * axis);

            // Subtract the illegal rotation
            return toLimits * rotation;
        }

        public static Quaternion LimitTwist(Quaternion rotation, Vector3 axis, Vector3 orthoAxis, float twistLimit)
        {
            twistLimit = Mathf.Clamp(twistLimit, 0, 180);
            if (twistLimit >= 180) return rotation;

            Vector3 normal = rotation * axis;
            Vector3 orthoTangent = orthoAxis;
            Vector3.OrthoNormalize(ref normal, ref orthoTangent);

            Vector3 rotatedOrthoTangent = rotation * orthoAxis;
            Vector3.OrthoNormalize(ref normal, ref rotatedOrthoTangent);

            Quaternion fixedRotation = Quaternion.FromToRotation(rotatedOrthoTangent, orthoTangent) * rotation;

            if (twistLimit <= 0) return fixedRotation;

            // Rotate from zero twist to free twist by the limited angle
            return Quaternion.RotateTowards(fixedRotation, rotation, twistLimit);
        }
    }
}
