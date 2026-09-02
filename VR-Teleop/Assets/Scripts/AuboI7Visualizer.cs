using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Builds and drives a lightweight AUBO i7_30 digital-twin hierarchy.
/// The exterior is intentionally stylized, while every joint origin, fixed RPY
/// transform and local +Z rotation axis matches the official i7_30 URDF values
/// used by teleop/aubo_i7_decoupled_ik.py.
/// </summary>
public sealed class AuboI7Visualizer : MonoBehaviour
{
    public const int JointCount = 6;

    private static readonly string[] JointNames =
    {
        "shoulder_joint",
        "upperArm_joint",
        "foreArm_joint",
        "wrist1_joint",
        "wrist2_joint",
        "wrist3_joint",
    };

    // Robot-frame metres, copied from aubo_i7_30.urdf.
    private static readonly Vector3[] JointOriginRobot =
    {
        new Vector3(0.0f, 0.0f, 0.122f),
        new Vector3(0.0f, 0.1215f, 0.0f),
        new Vector3(0.368f, 0.0f, 0.0f),
        new Vector3(0.316f, 0.0f, 0.0f),
        new Vector3(0.0f, 0.1025f, 0.0f),
        new Vector3(0.0f, -0.094f, 0.0f),
    };

    // URDF fixed-axis RPY in radians.
    private static readonly Vector3[] JointOriginRpyRobot =
    {
        new Vector3(0.0f, 0.0f, Mathf.PI),
        new Vector3(-Mathf.PI / 2.0f, -Mathf.PI / 2.0f, 0.0f),
        new Vector3(-Mathf.PI, 0.0f, 0.0f),
        new Vector3(Mathf.PI, 0.0f, Mathf.PI / 2.0f),
        new Vector3(-Mathf.PI / 2.0f, 0.0f, 0.0f),
        new Vector3(Mathf.PI / 2.0f, 0.0f, 0.0f),
    };

    private static readonly float[] NominalJointAnglesDegrees =
    {
        0.0f,
        -60.0f,
        -75.0f,
        0.0f,
        -75.0f,
        0.0f,
    };

    private static readonly float[] JointHousingRadii =
    {
        0.076f,
        0.074f,
        0.068f,
        0.058f,
        0.052f,
        0.047f,
    };

    private static readonly float[] LinkRadii =
    {
        0.057f,
        0.052f,
        0.047f,
        0.039f,
        0.034f,
    };

    [Header("Initial display posture")]
    [SerializeField]
    private float[] initialJointAnglesDegrees =
    {
        0.0f,
        -60.0f,
        -75.0f,
        0.0f,
        -75.0f,
        0.0f,
    };

    [Header("Stylized model colors")]
    [SerializeField] private Color linkColor = new Color(0.78f, 0.82f, 0.86f, 1.0f);
    [SerializeField] private Color jointColor = new Color(0.12f, 0.14f, 0.17f, 1.0f);
    [SerializeField] private Color accentColor = new Color(1.0f, 0.31f, 0.06f, 1.0f);

    private readonly Transform[] joints = new Transform[JointCount];
    private readonly Quaternion[] fixedJointRotations = new Quaternion[JointCount];

    private Transform generatedRoot;
    private Material linkMaterial;
    private Material jointMaterial;
    private Material accentMaterial;

    public long LastCommandSequence { get; private set; } = -1;
    public double LastPythonTimestamp { get; private set; }
    public string LastCommandMode { get; private set; } = "initial";
    public bool HasLiveJointState => LastCommandSequence >= 0;

    private void Awake()
    {
        BuildModel();
        ApplyJointAnglesDegrees(ValidatedInitialJointAngles());
    }

    public void ApplyJointState(
        IReadOnlyList<float> jointsRadians,
        long commandSequence,
        double pythonTimestamp,
        string commandMode
    )
    {
        if (jointsRadians == null || jointsRadians.Count != JointCount)
            throw new ArgumentException("AUBO i7 joint state must contain six angles.");
        if (commandSequence < 0)
            throw new ArgumentOutOfRangeException(nameof(commandSequence));
        if (double.IsNaN(pythonTimestamp) || double.IsInfinity(pythonTimestamp))
            throw new ArgumentOutOfRangeException(nameof(pythonTimestamp));

        BuildModel();
        for (int index = 0; index < JointCount; index++)
        {
            float radians = jointsRadians[index];
            if (float.IsNaN(radians) || float.IsInfinity(radians))
                throw new ArgumentException("AUBO i7 joint angles must be finite.");
            SetJointAngleDegrees(index, radians * Mathf.Rad2Deg);
        }

        LastCommandSequence = commandSequence;
        LastPythonTimestamp = pythonTimestamp;
        LastCommandMode = string.IsNullOrEmpty(commandMode) ? "unknown" : commandMode;
    }

    public Transform GetJointTransform(int jointIndex)
    {
        if (jointIndex < 0 || jointIndex >= JointCount)
            throw new ArgumentOutOfRangeException(nameof(jointIndex));
        BuildModel();
        return joints[jointIndex];
    }

    private void BuildModel()
    {
        if (generatedRoot != null)
            return;

        linkMaterial = CreateMaterial("AUBO Link", linkColor);
        jointMaterial = CreateMaterial("AUBO Joint", jointColor);
        accentMaterial = CreateMaterial("AUBO Accent", accentColor);

        GameObject rootObject = new GameObject("Generated i7_30 Kinematic Model");
        rootObject.layer = gameObject.layer;
        generatedRoot = rootObject.transform;
        generatedRoot.SetParent(transform, false);

        CreateCylinder(
            "base_link",
            generatedRoot,
            new Vector3(0.0f, 0.061f, 0.0f),
            Quaternion.identity,
            0.092f,
            0.122f,
            linkMaterial
        );
        CreateCylinder(
            "base_accent",
            generatedRoot,
            new Vector3(0.0f, 0.014f, 0.0f),
            Quaternion.identity,
            0.099f,
            0.020f,
            accentMaterial
        );

        Transform parent = generatedRoot;
        for (int index = 0; index < JointCount; index++)
        {
            GameObject jointObject = new GameObject(JointNames[index]);
            jointObject.layer = gameObject.layer;
            Transform joint = jointObject.transform;
            joint.SetParent(parent, false);
            joint.localPosition = RobotToUnityPosition(JointOriginRobot[index]);
            fixedJointRotations[index] = RobotRpyToUnityRotation(
                JointOriginRpyRobot[index]
            );
            joint.localRotation = fixedJointRotations[index];
            joints[index] = joint;

            float housingRadius = JointHousingRadii[index];
            CreateCylinder(
                $"{JointNames[index]}_housing",
                joint,
                Vector3.zero,
                Quaternion.identity,
                housingRadius,
                index < 3 ? 0.088f : 0.070f,
                jointMaterial
            );
            CreateBox(
                $"{JointNames[index]}_angle_marker",
                joint,
                new Vector3(housingRadius * 0.82f, 0.0f, 0.0f),
                new Vector3(
                    housingRadius * 0.45f,
                    index < 3 ? 0.094f : 0.076f,
                    0.014f
                ),
                accentMaterial
            );

            if (index < JointCount - 1)
            {
                Vector3 nextOrigin = RobotToUnityPosition(
                    JointOriginRobot[index + 1]
                );
                CreateCylinderBetween(
                    $"link_{index + 1}_to_{index + 2}",
                    joint,
                    Vector3.zero,
                    nextOrigin,
                    LinkRadii[index],
                    linkMaterial
                );
            }
            else
            {
                CreateCylinder(
                    "tool_flange",
                    joint,
                    new Vector3(0.0f, 0.033f, 0.0f),
                    Quaternion.identity,
                    0.041f,
                    0.066f,
                    linkMaterial
                );
                CreateCylinder(
                    "tool_flange_face",
                    joint,
                    new Vector3(0.0f, 0.069f, 0.0f),
                    Quaternion.identity,
                    0.048f,
                    0.008f,
                    accentMaterial
                );
            }

            parent = joint;
        }
    }

    private float[] ValidatedInitialJointAngles()
    {
        if (initialJointAnglesDegrees == null || initialJointAnglesDegrees.Length != JointCount)
            return NominalJointAnglesDegrees;

        for (int index = 0; index < JointCount; index++)
        {
            if (
                float.IsNaN(initialJointAnglesDegrees[index])
                || float.IsInfinity(initialJointAnglesDegrees[index])
            )
            {
                return NominalJointAnglesDegrees;
            }
        }
        return initialJointAnglesDegrees;
    }

    private void ApplyJointAnglesDegrees(IReadOnlyList<float> anglesDegrees)
    {
        for (int index = 0; index < JointCount; index++)
            SetJointAngleDegrees(index, anglesDegrees[index]);
    }

    private void SetJointAngleDegrees(int index, float angleDegrees)
    {
        // Mapping robot (x,y,z) -> Unity (x,z,y) changes handedness, so the
        // URDF local +Z rotation becomes a negative rotation about Unity local +Y.
        joints[index].localRotation =
            fixedJointRotations[index]
            * Quaternion.AngleAxis(-angleDegrees, Vector3.up);
    }

    private static Vector3 RobotToUnityPosition(Vector3 robotPosition)
    {
        return new Vector3(robotPosition.x, robotPosition.z, robotPosition.y);
    }

    private static Quaternion RobotRpyToUnityRotation(Vector3 robotRpy)
    {
        float roll = robotRpy.x;
        float pitch = robotRpy.y;
        float yaw = robotRpy.z;
        float cr = Mathf.Cos(roll);
        float sr = Mathf.Sin(roll);
        float cp = Mathf.Cos(pitch);
        float sp = Mathf.Sin(pitch);
        float cy = Mathf.Cos(yaw);
        float sy = Mathf.Sin(yaw);

        // R_robot = Rz(yaw) * Ry(pitch) * Rx(roll).
        float r00 = cy * cp;
        float r01 = cy * sp * sr - sy * cr;
        float r02 = cy * sp * cr + sy * sr;
        float r10 = sy * cp;
        float r11 = sy * sp * sr + cy * cr;
        float r12 = sy * sp * cr - cy * sr;
        float r20 = -sp;
        float r21 = cp * sr;
        float r22 = cp * cr;

        // Conjugate by the basis swap F:(x,y,z)->(x,z,y). Matrix columns are
        // the Unity right/up/forward vectors of the transformed link frame.
        Vector3 unityUp = new Vector3(r02, r22, r12);
        Vector3 unityForward = new Vector3(r01, r21, r11);
        return Quaternion.LookRotation(unityForward, unityUp);
    }

    private GameObject CreateCylinder(
        string objectName,
        Transform parent,
        Vector3 localPosition,
        Quaternion localRotation,
        float radius,
        float height,
        Material material
    )
    {
        GameObject visual = CreatePrimitive(PrimitiveType.Cylinder, objectName, parent);
        visual.transform.localPosition = localPosition;
        visual.transform.localRotation = localRotation;
        visual.transform.localScale = new Vector3(radius, height * 0.5f, radius);
        visual.GetComponent<Renderer>().sharedMaterial = material;
        return visual;
    }

    private GameObject CreateBox(
        string objectName,
        Transform parent,
        Vector3 localPosition,
        Vector3 localScale,
        Material material
    )
    {
        GameObject visual = CreatePrimitive(PrimitiveType.Cube, objectName, parent);
        visual.transform.localPosition = localPosition;
        visual.transform.localRotation = Quaternion.identity;
        visual.transform.localScale = localScale;
        visual.GetComponent<Renderer>().sharedMaterial = material;
        return visual;
    }

    private GameObject CreateCylinderBetween(
        string objectName,
        Transform parent,
        Vector3 start,
        Vector3 end,
        float radius,
        Material material
    )
    {
        Vector3 offset = end - start;
        float length = offset.magnitude;
        if (length <= Mathf.Epsilon)
            throw new InvalidOperationException("AUBO link length must be positive.");

        return CreateCylinder(
            objectName,
            parent,
            (start + end) * 0.5f,
            Quaternion.FromToRotation(Vector3.up, offset / length),
            radius,
            length,
            material
        );
    }

    private GameObject CreatePrimitive(
        PrimitiveType primitiveType,
        string objectName,
        Transform parent
    )
    {
        GameObject visual = GameObject.CreatePrimitive(primitiveType);
        visual.name = objectName;
        visual.layer = gameObject.layer;
        visual.transform.SetParent(parent, false);
        Collider generatedCollider = visual.GetComponent<Collider>();
        if (generatedCollider != null)
            Destroy(generatedCollider);
        return visual;
    }

    private static Material CreateMaterial(string materialName, Color color)
    {
        Shader shader =
            Shader.Find("Universal Render Pipeline/Lit")
            ?? Shader.Find("Standard")
            ?? Shader.Find("Unlit/Color");
        Material material = new Material(shader)
        {
            name = materialName,
            color = color,
        };
        if (material.HasProperty("_BaseColor"))
            material.SetColor("_BaseColor", color);
        return material;
    }

    private void OnDestroy()
    {
        DestroyMaterial(linkMaterial);
        DestroyMaterial(jointMaterial);
        DestroyMaterial(accentMaterial);
    }

    private static void DestroyMaterial(Material material)
    {
        if (material == null)
            return;
        if (Application.isPlaying)
            Destroy(material);
        else
            DestroyImmediate(material);
    }
}
