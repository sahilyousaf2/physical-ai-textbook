---
sidebar_label: 'Week 7: High-Fidelity Rendering in Unity'
---

# Week 7: High-Fidelity Rendering in Unity

## Learning Objectives

By the end of this week, you will be able to:
- Set up Unity for robotics visualization and simulation
- Create realistic 3D environments for humanoid robot testing
- Implement sensor simulation in Unity's rendering pipeline
- Integrate Unity with ROS 2 using ROS# or Unity Robotics Hub
- Develop custom shaders and materials for robot visualization
- Create interactive user interfaces for robot monitoring

## Overview

Unity provides high-fidelity rendering capabilities that complement physics simulation with photorealistic visualization. This week focuses on leveraging Unity's advanced graphics pipeline for robotics applications, including sensor simulation, realistic environment creation, and integration with ROS 2 for comprehensive digital twin capabilities.

## Unity Robotics Setup

### Installing Unity Robotics Hub
1. Download and install Unity Hub
2. Install Unity 2022.3 LTS or later
3. Install ROS# package or Unity Robotics Hub
4. Configure ROS connection settings

### Unity ROS Integration Options
- **ROS#**: Lightweight Unity package for ROS communication
- **Unity Robotics Hub**: Comprehensive toolkit with ROS bridges
- **Custom TCP/IP**: Direct socket communication for specific needs

### Basic ROS Connection in Unity
```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class UnityRobotController : MonoBehaviour
{
    private RosSocket rosSocket;
    private string rosBridgeServerUrl = "ws://192.168.1.100:9090"; // Update with your ROS bridge address

    void Start()
    {
        // Initialize ROS connection
        WebSocketNativeClient webSocket = new WebSocketNativeClient(new RosSocketFactory(rosBridgeServerUrl));
        rosSocket = new RosSocket(webSocket);

        // Subscribe to joint states
        rosSocket.Subscribe<sensor_msgs.JointState>("/joint_states", JointStateHandler);
    }

    void JointStateHandler(sensor_msgs.JointState jointState)
    {
        // Update robot joints based on received joint states
        for (int i = 0; i < jointState.name.Count; i++)
        {
            string jointName = jointState.name[i];
            float jointPosition = (float)jointState.position[i];
            UpdateJoint(jointName, jointPosition);
        }
    }

    void UpdateJoint(string jointName, float position)
    {
        // Find and update the corresponding joint in Unity
        Transform joint = transform.Find(jointName);
        if (joint != null)
        {
            // Apply rotation based on joint position
            joint.localRotation = Quaternion.Euler(0, position * Mathf.Rad2Deg, 0);
        }
    }
}
```

## Creating Realistic Robot Models in Unity

### Importing Robot Models
1. Export robot from CAD software as FBX or OBJ
2. Import into Unity with proper scale (typically 1 unit = 1 meter)
3. Set up materials and textures for realistic appearance
4. Configure colliders for physics interactions

### Robot Hierarchy Structure
```
HumanoidRobot/
├── BaseLink/
│   ├── Torso/
│   │   ├── Head/
│   │   │   └── Camera/
│   │   ├── LeftShoulder/
│   │   │   ├── LeftElbow/
│   │   │   │   └── LeftWrist/
│   │   ├── RightShoulder/
│   │   │   ├── RightElbow/
│   │   │   │   └── RightWrist/
│   │   ├── LeftHip/
│   │   │   ├── LeftKnee/
│   │   │   │   └── LeftAnkle/
│   │   └── RightHip/
│   │       ├── RightKnee/
│   │       │   └── RightAnkle/
└── Sensors/
    ├── LiDAR/
    ├── IMU/
    └── ForceSensors/
```

### Material Setup for Realistic Appearance
```csharp
using UnityEngine;

public class RobotMaterialController : MonoBehaviour
{
    [Header("Material Properties")]
    public Material robotBodyMaterial;
    public Material sensorMaterial;
    public Material jointMaterial;

    [Header("Visual Settings")]
    public Color bodyColor = Color.gray;
    public Color sensorColor = Color.blue;
    public Color jointColor = Color.red;

    void Start()
    {
        SetupMaterials();
    }

    void SetupMaterials()
    {
        if (robotBodyMaterial != null)
        {
            robotBodyMaterial.color = bodyColor;
            robotBodyMaterial.SetFloat("_Metallic", 0.7f);
            robotBodyMaterial.SetFloat("_Smoothness", 0.5f);
        }

        if (sensorMaterial != null)
        {
            sensorMaterial.color = sensorColor;
            sensorMaterial.SetFloat("_EmissionColor", sensorColor);
        }

        if (jointMaterial != null)
        {
            jointMaterial.color = jointColor;
            jointMaterial.SetFloat("_Metallic", 0.9f);
            jointMaterial.SetFloat("_Smoothness", 0.8f);
        }
    }
}
```

## Sensor Simulation in Unity

### Camera Sensor Simulation
```csharp
using UnityEngine;

public class UnityCameraSensor : MonoBehaviour
{
    [Header("Camera Settings")]
    public int imageWidth = 640;
    public int imageHeight = 480;
    public float fieldOfView = 60f;

    [Header("Output Settings")]
    public bool publishImages = true;
    public float publishRate = 30f; // Hz

    private Camera cam;
    private RenderTexture renderTexture;
    private Texture2D outputTexture;
    private float lastPublishTime;

    void Start()
    {
        SetupCamera();
        CreateRenderTexture();
    }

    void SetupCamera()
    {
        cam = GetComponent<Camera>();
        if (cam == null)
        {
            cam = gameObject.AddComponent<Camera>();
        }

        cam.fieldOfView = fieldOfView;
        cam.targetTexture = renderTexture;
    }

    void CreateRenderTexture()
    {
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        renderTexture.Create();

        outputTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
    }

    void Update()
    {
        if (publishImages && Time.time - lastPublishTime >= 1f / publishRate)
        {
            PublishImage();
            lastPublishTime = Time.time;
        }
    }

    void PublishImage()
    {
        // Set the camera's target texture
        RenderTexture.active = renderTexture;
        cam.Render();

        // Read pixels from render texture
        outputTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        outputTexture.Apply();

        // Convert to byte array and publish via ROS
        byte[] imageBytes = outputTexture.EncodeToJPG();

        // Here you would publish the image via ROS connection
        // This is a simplified example
        PublishToROS(imageBytes);
    }

    void PublishToROS(byte[] imageData)
    {
        // Implementation would send image data to ROS
        // via the established ROS connection
    }
}
```

### LiDAR Simulation
```csharp
using UnityEngine;
using System.Collections.Generic;

public class UnityLidarSensor : MonoBehaviour
{
    [Header("LiDAR Settings")]
    public int horizontalResolution = 360;
    public int verticalResolution = 1;
    public float minAngle = -Mathf.PI;
    public float maxAngle = Mathf.PI;
    public float maxRange = 10.0f;
    public float publishRate = 10f;

    [Header("Raycast Settings")]
    public LayerMask detectionLayers = -1;

    private float lastPublishTime;
    private List<float> ranges;

    void Start()
    {
        ranges = new List<float>(new float[horizontalResolution]);
    }

    void Update()
    {
        if (Time.time - lastPublishTime >= 1f / publishRate)
        {
            SimulateLidarScan();
            lastPublishTime = Time.time;
        }
    }

    void SimulateLidarScan()
    {
        float angleIncrement = (maxAngle - minAngle) / horizontalResolution;

        for (int i = 0; i < horizontalResolution; i++)
        {
            float angle = minAngle + (i * angleIncrement);
            Vector3 direction = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));

            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, maxRange, detectionLayers))
            {
                ranges[i] = hit.distance;
            }
            else
            {
                ranges[i] = maxRange;
            }
        }

        // Publish scan data via ROS
        PublishScanData();
    }

    void PublishScanData()
    {
        // Implementation would publish laser scan data to ROS
        // with appropriate message format
    }

    // Visualization helper
    void OnDrawGizmosSelected()
    {
        if (ranges != null)
        {
            Gizmos.color = Color.red;
            float angleIncrement = (maxAngle - minAngle) / horizontalResolution;

            for (int i = 0; i < horizontalResolution; i += 10) // Draw every 10th ray for performance
            {
                float angle = minAngle + (i * angleIncrement);
                Vector3 direction = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));
                float distance = ranges[i];

                Gizmos.DrawRay(transform.position, direction * distance);
            }
        }
    }
}
```

## Environment Creation

### Terrain Setup for Robotics
```csharp
using UnityEngine;

public class RoboticsEnvironment : MonoBehaviour
{
    [Header("Terrain Settings")]
    public float terrainWidth = 100f;
    public float terrainLength = 100f;
    public int resolution = 257; // Must be 2^n + 1

    [Header("Obstacle Settings")]
    public GameObject[] obstaclePrefabs;
    public int numObstacles = 10;

    private Terrain terrain;
    private TerrainData terrainData;

    void Start()
    {
        CreateTerrain();
        AddObstacles();
        ConfigurePhysics();
    }

    void CreateTerrain()
    {
        // Create terrain data
        terrainData = new TerrainData();
        terrainData.heightmapResolution = resolution;
        terrainData.size = new Vector3(terrainWidth, 10f, terrainLength); // height, width, length
        terrainData.SetDetailResolution(1024, 16);

        // Create terrain gameobject
        GameObject terrainObject = Terrain.CreateTerrainGameObject(terrainData);
        terrain = terrainObject.GetComponent<Terrain>();

        // Generate heightmap
        GenerateHeightmap();
    }

    void GenerateHeightmap()
    {
        float[,] heights = new float[resolution, resolution];

        // Generate a simple heightmap with hills and valleys
        for (int y = 0; y < resolution; y++)
        {
            for (int x = 0; x < resolution; x++)
            {
                float xCoord = (float)x / resolution * 5f;
                float yCoord = (float)y / resolution * 5f;

                // Generate Perlin noise-based terrain
                float height = Mathf.PerlinNoise(xCoord, yCoord) * 2f;

                // Add some variation
                height += Mathf.Sin(xCoord * 0.5f) * 0.5f;
                height += Mathf.Cos(yCoord * 0.3f) * 0.3f;

                heights[y, x] = height;
            }
        }

        terrainData.SetHeights(0, 0, heights);
    }

    void AddObstacles()
    {
        for (int i = 0; i < numObstacles; i++)
        {
            if (obstaclePrefabs.Length > 0)
            {
                GameObject obstaclePrefab = obstaclePrefabs[Random.Range(0, obstaclePrefabs.Length)];
                Vector3 position = new Vector3(
                    Random.Range(0, terrainWidth),
                    0,
                    Random.Range(0, terrainLength)
                );

                // Adjust Y position based on terrain height
                position.y = terrain.SampleHeight(position) + 0.5f;

                Instantiate(obstaclePrefab, position, Quaternion.identity);
            }
        }
    }

    void ConfigurePhysics()
    {
        // Add physics materials for realistic robot interaction
        PhysicMaterial physicsMat = new PhysicMaterial();
        physicsMat.staticFriction = 0.5f;
        physicsMat.dynamicFriction = 0.4f;
        physicsMat.bounciness = 0.1f;

        // Apply to terrain
        TerrainCollider terrainCollider = terrain.GetComponent<TerrainCollider>();
        terrainCollider.material = physicsMat;
    }
}
```

## Unity-ROS Integration Patterns

### Publisher Pattern
```csharp
using UnityEngine;
using RosSharp.RosBridgeClient;
using std_msgs = Messages.std_msgs;

public class UnityPublisher : MonoBehaviour
{
    [Header("Publisher Settings")]
    public string topicName = "/unity_robot_status";
    public float publishRate = 10f;

    private RosSocket rosSocket;
    private float lastPublishTime;

    void Start()
    {
        // Initialize ROS connection (code from earlier example)
        InitializeROS();
    }

    void Update()
    {
        if (Time.time - lastPublishTime >= 1f / publishRate)
        {
            PublishRobotStatus();
            lastPublishTime = Time.time;
        }
    }

    void PublishRobotStatus()
    {
        if (rosSocket != null)
        {
            std_msgs.String message = new std_msgs.String();
            message.data = $"Robot position: {transform.position}, time: {Time.time}";

            rosSocket.Publish(topicName, message);
        }
    }

    void InitializeROS()
    {
        // ROS connection initialization code
    }
}
```

## Practical Exercise: Unity Digital Twin

### Step 1: Create Robot Visualization Scene
1. Import your humanoid robot model
2. Set up materials and lighting
3. Create animation controllers for basic movements
4. Add sensor visualization components

### Step 2: Implement ROS Communication
1. Set up ROS connection
2. Subscribe to joint states from ROS
3. Publish sensor data back to ROS
4. Implement bidirectional communication

### Step 3: Environment Integration
1. Create a realistic environment
2. Add physics for realistic interactions
3. Implement sensor simulation
4. Add UI for monitoring robot status

## Hands-On Project: Complete Unity Digital Twin

Develop a complete Unity digital twin that includes:
1. A detailed humanoid robot model with proper materials
2. Real-time ROS communication for joint control
3. Sensor simulation for cameras and LiDAR
4. Interactive environment with obstacles
5. User interface for monitoring and control

## Assignment

1. Set up Unity with ROS integration
2. Create a detailed humanoid robot visualization
3. Implement sensor simulation (at least camera and LiDAR)
4. Create a realistic environment for robot testing
5. Document the integration process and any challenges encountered

## Resources

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS# Documentation](https://github.com/siemens/ros-sharp)
- [Unity Scripting API](https://docs.unity3d.com/ScriptReference/)
- [Unity Physics](https://docs.unity3d.com/Manual/PhysicsSection.html)

## Next Week Preview

Next week, we'll begin Module 3, focusing on NVIDIA Isaac Sim for photorealistic simulation. You'll learn to create advanced simulation environments that combine the physics accuracy of Gazebo with the visual fidelity of Unity, specifically optimized for AI training and development.