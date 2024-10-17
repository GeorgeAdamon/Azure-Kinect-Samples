using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Microsoft.Azure.Kinect.BodyTracking;
using System.Text;
using MathNet.Numerics.IntegralTransforms;

public class PuppetAvatar : MonoBehaviour
{
    public TrackerHandler KinectDevice;
    Dictionary<JointId, Quaternion> absoluteOffsetMap;
    Animator PuppetAnimator;
    public GameObject RootPosition;
    public Transform CharacterRootTransform;
    public float OffsetY;
    public float OffsetZ;

    public bool applyButterworthFilter;
    
    private Dictionary<JointId, Queue<Vector4>> jointRotationBuffers;
    public int bufferSize = 10; // Adjust this depending on desired smoothing (e.g., 1 second of data at 60 FPS)
    [Range(0,10)]
    public float lambda = 0.5f; // Decay parameter for exponential weighting (higher values = more emphasis on recent data)
   
    private static HumanBodyBones MapKinectJoint(JointId joint)
    {
        // https://docs.microsoft.com/en-us/azure/Kinect-dk/body-joints
        switch (joint)
        {
            case JointId.Pelvis: return HumanBodyBones.Hips;
            case JointId.SpineNavel: return HumanBodyBones.Spine;
            case JointId.SpineChest: return HumanBodyBones.Chest;
            case JointId.Neck: return HumanBodyBones.Neck;
            case JointId.Head: return HumanBodyBones.Head;
            case JointId.HipLeft: return HumanBodyBones.LeftUpperLeg;
            case JointId.KneeLeft: return HumanBodyBones.LeftLowerLeg;
            case JointId.AnkleLeft: return HumanBodyBones.LeftFoot;
            case JointId.FootLeft: return HumanBodyBones.LeftToes;
            case JointId.HipRight: return HumanBodyBones.RightUpperLeg;
            case JointId.KneeRight: return HumanBodyBones.RightLowerLeg;
            case JointId.AnkleRight: return HumanBodyBones.RightFoot;
            case JointId.FootRight: return HumanBodyBones.RightToes;
            case JointId.ClavicleLeft: return HumanBodyBones.LeftShoulder;
            case JointId.ShoulderLeft: return HumanBodyBones.LeftUpperArm;
            case JointId.ElbowLeft: return HumanBodyBones.LeftLowerArm;
            case JointId.WristLeft: return HumanBodyBones.LeftHand;
            case JointId.ClavicleRight: return HumanBodyBones.RightShoulder;
            case JointId.ShoulderRight: return HumanBodyBones.RightUpperArm;
            case JointId.ElbowRight: return HumanBodyBones.RightLowerArm;
            case JointId.WristRight: return HumanBodyBones.RightHand;
            default: return HumanBodyBones.LastBone;
        }
    }
    private void Start()
    {
        PuppetAnimator = GetComponent<Animator>();
        Transform _rootJointTransform = CharacterRootTransform;

        absoluteOffsetMap = new Dictionary<JointId, Quaternion>();
        jointRotationBuffers = new Dictionary<JointId, Queue<Vector4>>();
        
        for (int i = 0; i < (int)JointId.Count; i++)
        {
            HumanBodyBones hbb = MapKinectJoint((JointId)i);
            if (hbb != HumanBodyBones.LastBone)
            {
                Transform transform = PuppetAnimator.GetBoneTransform(hbb);
                Quaternion absOffset = GetSkeletonBone(PuppetAnimator, transform.name).rotation;
                // find the absolute offset for the tpose
                while (!ReferenceEquals(transform, _rootJointTransform))
                {
                    transform = transform.parent;
                    absOffset = GetSkeletonBone(PuppetAnimator, transform.name).rotation * absOffset;
                }
                absoluteOffsetMap[(JointId)i] = absOffset;
                
                jointRotationBuffers[(JointId)i] = new Queue<Vector4>(new Vector4[bufferSize]);
            }
        }
    }

    private static SkeletonBone GetSkeletonBone(Animator animator, string boneName)
    {
        int count = 0;
        StringBuilder cloneName = new StringBuilder(boneName);
        cloneName.Append("(Clone)");
        foreach (SkeletonBone sb in animator.avatar.humanDescription.skeleton)
        {
            if (sb.name == boneName || sb.name == cloneName.ToString())
            {
                return animator.avatar.humanDescription.skeleton[count];
            }
            count++;
        }
        return new SkeletonBone();
    }

    // Update is called once per frame
    private void LateUpdate()
    {
        for (int j = 0; j < (int)JointId.Count; j++)
        {
            JointId jointId = (JointId)j;
            
            if (MapKinectJoint((JointId)j) != HumanBodyBones.LastBone && absoluteOffsetMap.ContainsKey((JointId)j))
            {
                // Get the current rotation and add it to the buffer
                Quaternion currentRotation = KinectDevice.absoluteJointRotations[j];

                // get the absolute offset
                Quaternion absOffset = absoluteOffsetMap[(JointId)j];
                Transform finalJoint = PuppetAnimator.GetBoneTransform(MapKinectJoint((JointId)j));
                
                if (applyButterworthFilter)
                {
                    Vector4 currentRotationVec = new Vector4(currentRotation.x, currentRotation.y, currentRotation.z, currentRotation.w);
                    Queue<Vector4> buffer = jointRotationBuffers[jointId];
                
                    if (buffer.Count >= bufferSize)
                    {
                        buffer.Dequeue();
                    }
                    buffer.Enqueue(currentRotationVec);
                
                    // Apply spectral smoothing to the buffer
                    //Quaternion smoothedRotation =  PerformSpectralSmoothing(buffer.ToArray(), cutoffFrequency, samplingFrequency);

                    // Apply Butterworth high-pass filter to the buffer
                    //Vector4 filteredRotationVec = ApplyButterworthHighPassFilter(buffer.ToArray(), cutoffFrequency, samplingFrequency);

                    // Convert back to Quaternion
                    //Quaternion smoothedRotation = new Quaternion(filteredRotationVec.x, filteredRotationVec.y, filteredRotationVec.z, filteredRotationVec.w);


                    Quaternion smoothedRotation = CalculateWeightedMovingAverageWithExponentialDecay(buffer, lambda);
                    
                    finalJoint.rotation = absOffset * Quaternion.Inverse(absOffset) * smoothedRotation * absOffset;

                }
                else
                {
                    finalJoint.rotation = absOffset * Quaternion.Inverse(absOffset) * KinectDevice.absoluteJointRotations[j] * absOffset;
                }
                
                if (j == 0)
                {
                    // character root plus translation reading from the kinect, plus the offset from the script public variables
                    finalJoint.position = CharacterRootTransform.position + 
                                          new Vector3(RootPosition.transform.localPosition.x, 
                                              RootPosition.transform.localPosition.y + OffsetY, 
                                              RootPosition.transform.localPosition.z - OffsetZ);
                }
            }
        }
    }
    
    private Quaternion PerformSpectralSmoothing(Vector4[] buffer, float frequencyCutoff, float samplingFrequency)
    {
        // Convert the quaternion to Vector4 for FFT
        Vector4[] vectorBuffer = buffer;//buffer.Select(q => new Vector4(q.x, q.y, q.z, q.w)).ToArray();

        // Perform FFT on each component
        double[] real = new double[bufferSize];
        double[] imag = new double[bufferSize];

        // Apply FFT for each component and filter out high frequencies
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < bufferSize; j++)
            {
                real[j] = vectorBuffer[j][i];
                imag[j] = 0;
            }

            // Perform FFT
            Fourier.Forward(real, imag, FourierOptions.Matlab);

            // Apply low-pass filter by zeroing out high frequencies
            //int cutoff = bufferSize / 4; // Adjust cutoff as needed
            
            // Calculate cutoff index based on frequencyCutoff and samplingFrequency
            int cutoff = Mathf.RoundToInt((frequencyCutoff / samplingFrequency) * bufferSize);

            for (int j = cutoff; j < bufferSize - cutoff; j++)
            {
                real[j] = 0;
                imag[j] = 0;
            }

            // Perform Inverse FFT
            Fourier.Inverse(real, imag, FourierOptions.Matlab);

            // Store the smoothed value back
            for (int j = 0; j < bufferSize; j++)
            {
                vectorBuffer[j][i] = (float)real[j];
            }
        }

        // Convert back to Quaternion (use the latest smoothed value)
        Vector4 smoothedVec = vectorBuffer[bufferSize - 1];
        return new Quaternion(smoothedVec.x, smoothedVec.y, smoothedVec.z, smoothedVec.w);
    }
    
    private static Vector4 ApplyButterworthHighPassFilter(Vector4[] buffer, float cutoffFrequency, float samplingFrequency)
    {
        int n = buffer.Length;
        Vector4 result = buffer[n - 1];

        if (n < 2) return result;

        // Butterworth filter parameters (second-order)
        float omega = 2.0f * Mathf.PI * cutoffFrequency / samplingFrequency;
        float tanOmega = Mathf.Tan(omega / 2.0f);
        float a0 = 1.0f / (1.0f + Mathf.Sqrt(2.0f) * tanOmega + tanOmega * tanOmega);
        float a1 = -2.0f * a0;
        float a2 = a0;
        float b1 = 2.0f * (tanOmega * tanOmega - 1.0f) * a0;
        float b2 = (1.0f - Mathf.Sqrt(2.0f) * tanOmega + tanOmega * tanOmega) * a0;

        Vector4 prevOutput = buffer[n - 2]; // Last output value (y[n-1])
        Vector4 prevPrevOutput = buffer[n - 3 >= 0 ? n - 3 : n - 2]; // y[n-2], fallback if not enough data
        Vector4 prevInput = buffer[n - 2]; // Last input value (x[n-1])
        Vector4 prevPrevInput = buffer[n - 3 >= 0 ? n - 3 : n - 2]; // x[n-2]

        // Apply Butterworth filter equation
        result = a0 * result + a1 * prevInput + a2 * prevPrevInput + b1 * prevOutput + b2 * prevPrevOutput;

        return result;
    }
    
    
    private Quaternion CalculateWeightedMovingAverageWithExponentialDecay(Queue<Vector4> buffer, float lambda)
    {
        if (buffer.Count == 0)
        {
            return Quaternion.identity; // If the buffer is empty, return identity quaternion
        }

        Quaternion[] quaternions = buffer.Select(v => new Quaternion(v.x, v.y, v.z, v.w)).ToArray();
        int count = quaternions.Length;

        // Calculate weights using the exponential decay function
        float[] weights = new float[count];
        float totalWeight = 0.0f;

        for (int i = 0; i < count; i++)
        {
            weights[i] = Mathf.Exp(-lambda * i); // Calculate weight for each element in the buffer
            totalWeight += weights[i];
        }

        // Normalize weights so that their sum is equal to 1
        for (int i = 0; i < count; i++)
        {
            weights[i] /= totalWeight;
        }

        // Apply SLERP with weights to calculate the weighted average
        Quaternion weightedAverage = quaternions[0]; // Start with the most recent quaternion

        for (int i = 1; i < count; i++)
        {
            weightedAverage = Quaternion.Slerp(weightedAverage, quaternions[i], weights[i]);
        }

        return weightedAverage;
    }

}
