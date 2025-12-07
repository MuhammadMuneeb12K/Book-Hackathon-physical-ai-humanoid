# Unity Human-Robot Interaction Scene Description

This document outlines the conceptual setup for a Human-Robot Interaction (HRI) scene in Unity, complementing the Gazebo simulation. Due to the complexity of a full Unity project setup via CLI, this description serves as a guide for manual implementation within the Unity editor.

## Goal

To create a visually rich and interactive Unity scene where a simulated humanoid robot (whose behavior is driven by Gazebo/ROS 2) can interact with a human user. The scene should demonstrate high-fidelity rendering and responsive user interface elements.

## Scene Elements

1.  **Environment:**
    *   A realistic indoor environment (e.g., living room, office space) with furniture, textures, and lighting that enhances immersion.
    *   Consider using Unity's High Definition Render Pipeline (HDRP) or Universal Render Pipeline (URP) for advanced visual effects.
2.  **Humanoid Robot Model:**
    *   Import a detailed 3D model of the humanoid robot. This model should be visually accurate and ideally capable of receiving animation data from the ROS 2 side (e.g., joint states from `joint_state_publisher`).
    *   The model should be positioned and scaled appropriately within the Unity scene.
3.  **Interactive Objects:**
    *   Objects that the robot can interact with (e.g., a cup on a table, a button on a wall). These should have colliders and potentially scripts for interaction feedback.
4.  **User Interface (UI) Elements:**
    *   **Robot Status Display:** Text or visual indicators showing the robot's current state (e.g., "Idle," "Moving," "Error").
    *   **Control Panel:** Buttons or sliders for sending high-level commands to the robot (e.g., "Go to table," "Pick up object," "Wave"). These commands would be translated into ROS 2 messages or actions via an integration layer.
    *   **Feedback Display:** Text area to show robot responses or system messages.
5.  **Camera Setup:**
    *   A user-controlled camera (e.g., orbit camera, first-person camera) to allow exploration of the scene.
    *   Optionally, a robot-mounted camera feed (simulated or real) displayed within the UI.

## Unity-ROS 2 Integration (Conceptual)

To link Unity with the Gazebo/ROS 2 environment:

1.  **ROS-TCP-Endpoint:** Utilize a package like `Unity-Technologies/ROS-TCP-Endpoint` to establish a TCP connection between Unity and the ROS 2 system.
2.  **Message Exchange:**
    *   **Unity to ROS 2:** UI button presses or user interactions in Unity send ROS 2 messages (e.g., `String` messages for high-level commands, `geometry_msgs/Pose` for goal locations) to control the robot.
    *   **ROS 2 to Unity:** Robot joint states (`sensor_msgs/JointState`), sensor data (e.g., `sensor_msgs/Image` for robot camera feed, `sensor_msgs/LaserScan` for LiDAR), and overall status are published by ROS 2 nodes and subscribed to by Unity scripts.
3.  **Robot Animation:** Unity scripts parse `JointState` messages from ROS 2 and apply the received joint angles to the humanoid robot model's Animator or individual joint transforms, enabling real-time visualization of robot movement.
4.  **Object Synchronization:** If the robot manipulates objects in Gazebo, their positions and orientations could be synchronized to the Unity scene via ROS 2 messages for a consistent visual representation.

## Setup Steps (Manual in Unity Editor)

1.  **Create New Unity Project:** Start a new 3D (HDRP/URP recommended) Unity project.
2.  **Import 3D Assets:** Bring in humanoid robot model, environment models, and interactive objects.
3.  **Configure Rendering:** Set up lighting, post-processing, and materials for a high-fidelity look.
4.  **Implement UI:** Design and implement the UI elements using Unity's UI Toolkit or Canvas system.
5.  **Integrate ROS-TCP-Endpoint:** Install the `ROS-TCP-Endpoint` package and configure communication.
6.  **Write C# Scripts:**
    *   Scripts to send commands to ROS 2 based on UI interaction.
    *   Scripts to subscribe to ROS 2 topics (e.g., `JointState`) and update the robot model's animations/transforms.
    *   Scripts to visualize sensor data or robot status.

This description provides a conceptual framework. A fully functional Unity HRI scene requires hands-on development within the Unity editor and detailed scripting.
