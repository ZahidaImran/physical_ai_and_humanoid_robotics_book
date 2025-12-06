# Glossary of Terms

This glossary provides definitions for key technical terms and acronyms used throughout the "Physical AI and Humanoid Robotics" book.

## A
**Action**: In ROS 2, a long-running, asynchronous communication mechanism that provides feedback and can be preempted.

## C
**`colcon`**: The command-line tool used to build ROS 2 packages within a workspace.
**Computer Vision**: A field of artificial intelligence that enables computers to "see" and interpret images and videos.

## D
**Digital Twin**: A virtual representation that serves as the real-time digital counterpart of a physical object or process, allowing for continuous interaction and data exchange.
**Distribution (ROS 2)**: A bundled set of ROS 2 packages released together, typically with a specific name (e.g., Humble, Iron) and support lifecycle.
**Domain Randomization**: A technique used in simulation to train robust AI models by varying environmental parameters (textures, lighting, object positions, physics properties) to improve real-world generalization.
**Docusaurus**: A modern static website generator for documentation.
**Durability (QoS)**: A ROS 2 Quality of Service policy that determines whether new subscribers receive old messages or only messages published after they subscribe.

## G
**Gazebo**: A robust, open-source 3D robot simulator that provides an accurate physics engine and integration with ROS 2.
**Grounding (VLA)**: The critical process of connecting abstract linguistic symbols (words, phrases) to their referents in the robot's physical environment and its capabilities.

## H
**History (QoS)**: A ROS 2 Quality of Service policy that determines how many messages are stored by a publisher for new subscribers.

## I
**Isaac Gym**: A PyTorch-based physics simulation framework from NVIDIA that enables massive parallelization of reinforcement learning environments on GPUs.
**Isaac SDK**: NVIDIA's software development kit for AI-powered robotics applications, providing core functionalities for perception, navigation, and manipulation.
**Isaac Sim**: NVIDIA's high-fidelity, GPU-accelerated robotics simulator built on the Omniverse platform, used for developing, testing, and training AI-powered robots.
**IMU (Inertial Measurement Unit)**: A sensor that measures a body's specific force, angular rate, and often the orientation of the body.

## J
**Jetson Platform**: NVIDIA's line of compact, powerful AI supercomputers designed for edge devices like robots.
**Joint**: In URDF, an element that connects two `link` elements, defining their relative motion.

## L
**Launch File**: A Python script in ROS 2 used to automate the startup of multiple nodes and configure them.
**LiDAR (Light Detection and Ranging)**: A remote sensing method that uses pulsed laser light to measure distances to the Earth.
**Link**: In URDF, a rigid body element that represents a part of the robot.

## M
**Multimodal Intelligence**: The ability of an AI system to process and understand information from multiple modalities, such as vision and language.

## N
**Natural Language Generation (NLG)**: A subfield of NLP focused on generating human-like text from structured data.
**Natural Language Processing (NLP)**: A field of AI that enables computers to understand, interpret, and generate human language.
**Natural Language Understanding (NLU)**: A subfield of NLP focused on extracting meaning and intent from natural language input.
**Node**: The fundamental processing unit in ROS 2, a process that performs computation.

## O
**Object Detection**: A computer vision task that involves identifying and localizing objects within an image or video.
**Omniverse**: NVIDIA's platform for building and operating metaverse applications, which Isaac Sim leverages.

## P
**Parameter**: In ROS 2, a configuration value within a node that can be set and changed at runtime.
**Perception**: A robot's ability to gather and interpret information from its environment, often using sensors and AI.

## Q
**Quality of Service (QoS)**: A set of policies in ROS 2 that defines how messages are sent and received, influencing reliability, durability, and history.

## R
**`rclpy`**: The Python client library for ROS 2.
**Reinforcement Learning (RL)**: A type of machine learning where an agent learns optimal behaviors through trial and error by interacting with an environment and receiving rewards or penalties.
**Reliability (QoS)**: A ROS 2 Quality of Service policy that determines whether message delivery is guaranteed (`RELIABLE`) or best-effort (`BEST_EFFORT`).
**Referring Expressions**: Natural language descriptions used to identify specific objects in a visual scene.
**ROS (Robot Operating System)**: A flexible framework for writing robot software.
**ROS 2**: The second generation of the Robot Operating System, designed for real-time performance, multi-robot systems, and improved security.
**`ros2_control`**: A ROS 2 framework for controlling robots, providing a standardized interface between high-level commands and robot hardware/simulated hardware.
**RTX**: NVIDIA's real-time ray tracing technology, used in Isaac Sim for photorealistic rendering.
**`rviz2`**: A 3D visualization tool for ROS 2.

## S
**SDF (Simulation Description Format)**: An XML format used by Gazebo to describe everything in a simulation, including robots, environments, and plugins.
**Segmentation**: A computer vision task that involves partitioning an image into multiple segments, often to identify objects or regions.
**Service**: In ROS 2, a synchronous request-response communication mechanism between nodes.
**SLAM (Simultaneous Localization and Mapping)**: A computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it.
**`std_msgs`**: A common ROS 2 package containing standard message types (e.g., `String`, `Empty`).
**Synthetic Data**: Data that is artificially generated in a simulation, mimicking real-world data but with perfect ground truth labels, used for AI training.

## T
**Task Planning**: The process of generating a sequence of actions or operations to achieve a specific goal.
**`termios`**: A Python module (POSIX-specific) for controlling terminal I/O settings, often used for keyboard input.
**Topic**: In ROS 2, an asynchronous publish-subscribe communication mechanism for sending messages between nodes.
**Transformer-based Models**: AI architectures that leverage self-attention mechanisms, widely used in NLP and increasingly in multimodal (VLA) tasks.
**`Twist` (geometry_msgs/msg)**: A common ROS 2 message type used to represent linear and angular velocity commands for robots.

## U
**Unity**: A commercial game engine used for creating 3D interactive applications, including robotics simulations.
**URDF (Unified Robot Description Format)**: An XML format for describing the kinematic and dynamic properties of a robot.
**USD (Universal Scene Description)**: An open-source 3D scene description format developed by Pixar, which NVIDIA Omniverse and Isaac Sim are built upon.

## V
**VLA (Vision-Language-Action)**: A field of AI that integrates computer vision, natural language processing, and robot control to enable robots to understand and act upon human commands.
**Visual Question Answering (VQA)**: A task where an AI model answers natural language questions about the content of an image.
