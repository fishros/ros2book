3.4.2 Custom Communication Interface
====================================

After opening the terminal, enter the command from Code Listing 3-25 in the `topic_practice_ws/src` directory.

**Listing 3-25: Create an interface package**

.. code-block:: bash

   ros2 pkg create status_interfaces --build-type ament_cmake --dependencies rosidl_default_generators builtin_interfaces --license Apache-2.0
   ---
   going to create a new package
   package name: status_interfaces
   destination directory: /home/fishros/chapt3/topic_practice_ws/src
   package format: 3
   version: 0.0.0
   description: TODO: Package description
   maintainer: ['fishros <fishros@todo.todo>']
   licenses: ['Apache-2.0']
   build type: ament_cmake
   dependencies: ['rosidl_default_generators','builtin_interfaces']
   ...
The above instructions are used to create a `status_interfaces` package (with the build type `ament_cmake`) and add two dependencies to it: `builtin_interfaces` and `rosidl_default_generators`.

`builtin_interfaces` is an existing message interface package in ROS 2, which provides the `Time` interface for recording timestamps. `rosidl_default_generators` is a module used to convert custom message files into C++ and Python source code.

In ROS 2, topic message definition files must be placed in the `msg` directory of the package. The filenames must start with an uppercase letter and can only contain alphanumeric characters.

Next, create a `msg` directory under the package and add a new file named `SystemStatus.msg` in that directory. Then, write the content of **Code Listing 3-26** into the file.


**Listing 3-26: src/status_interfaces/msg/SystemStatus.msg**

.. code-block:: bash

   builtin_interfaces/Time stamp  # Timestamp
   string host_name              # System name
   float32 cpu_percent           # CPU usage
   float32 memory_percent        # Memory usage
   float32 memory_total          # Total memory
   float32 memory_available      # Available memory
   float64 net_sent              # Total data sent over network
   float64 net_recv              # Total data received over network

This gives us a message interface definition file. As we can see, the syntax for defining message files is similar to defining variables in C++, using a type + name format, where text following # is a comment.

The timestamp is recorded using the Time type from the builtin_interfaces package. All other data types consist of string, float32, and float64, which are ROS 2's built-in primitive types. Besides these three, ROS 2 also defines nine additional data types, as shown in Code Listing 3-27.

**Listing 3-27: The nine data types supported by ROS 2 message interfaces.**

.. code-block:: bash

   - bool
   - byte
   - char
   - float32, float64
   - int8, uint8
   - int16, uint16
   - int32, uint32
   - int6, uint64
   - string

After defining the data interface file, the next step is to register it in CMakeLists.txt, declaring it as a message interface file and adding the builtin_interfaces dependency. Once completed, the CMakeLists.txt code will appear as shown in Code Listing 3-28.

**Listing 3-28: src/status_interfaces/CMakeLists.txt**

.. code-block:: bash

   # Find dependencies
   find_package(ament_cmake REQUIRED)
   find_package(rosidl_default_generators REQUIRED)
   find_package(builtin_interfaces REQUIRED)

   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/SystemStatus.msg"
     DEPENDENCIES builtin_interfaces
   )
   ...
   ament_package

Since the custom message interface uses `builtin_interfaces/Time` to represent time, the `find_package` command is used in the above file to locate `builtin_interfaces`, and `builtin_interfaces` is added as a dependency in `rosidl_generate_interfaces`.

In addition to modifying `CMakeLists.txt`, it is recommended to declare the dependency in the package manifest file `package.xml`. After completing these changes, the main file contents are as shown in Code Listing 3-29.

**Listing 3-29: topic_practices_ws/src/status_interfaces/package.xml**

.. code-block:: xml
   ...
   <license>Apache-2.0</license>
   <member_of_group>rosidl_interface_packages</member_of_group>
   <buildtool_depend>ament_cmake</buildtool_depend>
   ...

Adding `member_of_group` in `package.xml` is to declare that this package is a message interface package, making it easier for ROS 2 to perform additional processing.

Next, proceed to build the package. After the build is complete, you can use the command in **Code Listing 3-30** to verify whether the message interface was successfully built.

**Listing 3-30: Check if the interface was built successfully**

.. code-block:: bash

   $source install/setup.bash
   $ros2 interface show status_interfaces/msg/SystemStatus
   ---

   builtin_interfaces/Time stamp  # Timestamp
           int32 sec
           uint32 nanosec
   string host_name              # System name
   float32 cpu_percent           # CPU usage
   float32 memory_percent        # Memory usage
   float32 memory_total          # Total memory
   float32 memory_available      # Available memory
   float64 net_sent              # Total data sent over network
   float64 net_recv              # Total data received over network

It should be noted that before using the command line to check, you should also use the `source` command to inform ROS 2 of the package's installation location.

In addition to using command-line tools to verify whether the build was successful, you can also check whether C++ header files have been generated in the `install/status_interfaces/include/` directory and whether the Python library for `status_interfaces` has been generated in the `install/status_interfaces/local/lib/python3.10/dist-packages/` directory.

At this point, the work of creating the custom message interface is complete. With the message interface ready, we can now proceed to write code and use the interface to transmit data.