.. _ros2_robot_development_features:

1.1.4 Features of ROS 2 Robot Development
==========================================

Get your gear ready, and you are about to embark on a journey into robot development based on ROS 2. But hold on a moment! Before we set off, let's take a look at what ROS 2 has prepared for us to help us build a powerful robot.

1.1.4.1 Four Core Communication Mechanisms
-------------------------------------------

The four core communication mechanisms of ROS 2 are Topics, Services, Parameters, and Actions.

* **Topics**: Topic communication refers to a Publish-Subscribe communication pattern. Publishers send messages to a specific topic, and Subscribers receive the messages by subscribing to that topic. The data flow is unidirectional. For example, we can publish the robot's position information to a topic with a specific name, and by subscribing to that topic, we can obtain the robot's position information.
* **Services**: Compared to topic communication, service communication is bidirectional. Service communication consists of a Service Server and a Service Client. The client sends requests to the server, which processes the requests and returns the results.
* **Actions**: Action communication also involves an Action Client and an Action Server. Unlike service communication, the Action Server can provide feedback on the progress of processing the client's request and can be canceled at any time. Action communication is often used for complex robot behaviors.
* **Parameters**: Parameter communication is mainly used for setting and reading robot parameters.

1.1.4.2 Rich Debugging Tools
----------------------------

In addition to the core communication mechanisms, ROS 2 also provides a wide range of visualization and debugging tools:

* **RViz**: For 3D visualization.
* **rqt series tools**: For visualizing charts, images, and other data.
* **ros2 bag**: For recording and replaying data.

1.1.4.3 Modeling and Kinematics Tools
-------------------------------------

ROS 2 provides essential tools for robot development:

* **TF tool**: For kinematics coordinate transformation and management.
* **URDF file format**: For describing the structure, joints, sensors, and other information of a robot.

1.1.4.4 Strong Open-Source Community and Application Frameworks
---------------------------------------------------------------

In addition to the core tools provided by ROS 2 itself, the open-source community has developed a rich set of tools and frameworks based on ROS 2:

* **Gazebo**: A powerful simulation tool.
* **Navigation 2**: An application framework for mobile robot navigation.
* **Moveit 2**: An application framework for robotic arm motion planning.

With such a rich set of tools provided by ROS 2, you don't have to worry about not being able to develop a powerful robot. That's it for the theoretical introduction. I believe you are already eager to start your journey in robot development. So let's get started now!