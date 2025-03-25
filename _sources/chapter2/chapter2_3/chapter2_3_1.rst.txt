2.3.1 Writing Nodes in a Package
================================

Open the integrated terminal in VS Code, navigate to the `chapt2` directory, and enter the command shown in Listing 2-21.

**Listing 2-21: Creating a C++ Package**

.. code-block:: bash

   $ ros2 pkg create demo_cpp_pkg --build-type ament_cmake --license Apache-2.0
   ---
   going to create a new package
   package name: demo_cpp_pkg
   destination directory: /home/fishros/chapt2
   package format: 3
   version: 0.0.0
   description: TODO: Package description
   maintainer: ['fishros <fishros@todo.todo>']
   licenses: ['Apache-2.0']
   build type: ament_cmake
   dependencies: []
   creating folder ./demo_cpp_pkg
   creating ./demo_cpp_pkg/package.xml
   creating source and include folder
   creating folder ./demo_cpp_pkg/src
   creating folder ./demo_cpp_pkg/include/demo_cpp_pkg
   creating ./demo_cpp_pkg/CMakeLists.txt

The `ros2 pkg create` command is used to create a package. Here, `demo_cpp_pkg` is the name of the package, `--build-type ament_cmake` specifies that the package’s build type is `ament_cmake`, and `--license Apache-2.0` declares the open-source license for the package. From the log, it’s clear that this command creates a `demo_cpp_pkg` folder in the current directory and generates some default files and folders within it. Expand this folder in the VS Code Explorer on the left, as shown in Figure 2-11.

.. figure:: figure2-11.png
    :alt: C++ Package Directory Structure
    :align: center

    Figure 2-11 C++ Package Directory Structure

In the next section, we’ll analyze the structure of this package. For now, you only need to know that you write nodes in the `src` directory. Add `cpp_node.cpp` under `demo_cpp_pkg/src`, and then enter the code shown in Listing 2-22 in the file.

**Listing 2-22: A Simple C++ Node**

.. code-block:: cpp

   #include "rclcpp/rclcpp.hpp"

   int main(int argc, char **argv)
   {
       rclcpp::init(argc, argv);
       auto node = std::make_shared<rclcpp::Node>("cpp_node");
       RCLCPP_INFO(node->get_logger(), "Hello C++ Node!");
       rclcpp::spin(node);
       rclcpp::shutdown();
       return 0;
   }

You’re not mistaken—Listing 2-22 is the same code from Section 1.4.4, which you can use directly without adding extra code. After writing the code, you’ll also need to register the node and add dependencies. Edit `CMakeLists.txt`, and the final additions and their locations are shown in Listing 2-23.

**Listing 2-23: chapt2/demo_cpp_pkg/CMakeLists.txt**

.. code-block:: cmake

   cmake_minimum_required(VERSION 3.8)
   …
   find_package(ament_cmake REQUIRED)
   # Uncomment the following section in order to fill in
   # further dependencies manually.
   # 1. Find rclcpp headers and libraries
   find_package(rclcpp REQUIRED)
   # 2. Add executable cpp_node
   add_executable(cpp_node src/cpp_node.cpp)
   # 3. Add dependencies for cpp_node
   ament_target_dependencies(cpp_node rclcpp)
   # 4. Copy cpp_node to the install directory
   install(TARGETS
   cpp_node
   DESTINATION lib/${PROJECT_NAME}
   )
   …
   ament_package()

In Listing 2-23, after adding `find_package` and `add_executable` to find dependencies and add the executable, we use the `ament_target_dependencies` directive provided by `ament_cmake` to add dependencies. Finally, the `install` directive is added to copy the compiled executable to the `install/demo_cpp_pkg/lib/demo_cpp_pkg` directory, so that `ros2 run` can find the node.

When creating the C++ package, the selected build type is `ament_cmake`, which is essentially a superset of CMake. `ament_cmake` adds some more convenient directives on top of CMake’s instruction set. In Listing 2-23, you can see the `find_package(ament_cmake REQUIRED)` directive, which is automatically added when creating an `ament_cmake` package. This allows you to use `ament`-related directives, such as `ament_target_dependencies` and `ament_package`.

The last line in Listing 2-23 is `ament_package()`. This directive collects information from `CMakeLists.txt`, generates indexes, and performs related configurations. Therefore, this directive must be called at the end of `CMakeLists.txt` for every `ament_cmake`-type package.

Before building the package, you also need to add a dependency declaration for `rclcpp` in the manifest file `package.xml`. The complete declaration is shown in Listing 2-24.

**Listing 2-24: chapt2/demo_cpp_pkg/package.xml**

.. code-block:: xml

   <?xml version="1.0"?>
   …
     <license>Apache-2.0</license>
     <depend>rclcpp</depend>
     <test_depend>ament_copyright</test_depend>
   …
   </package>

The `<depend>rclcpp</depend>` line declares that the current package depends on the `rclcpp` library. After completing these steps, you can build the package using the command in Listing 2-25.

**Listing 2-25: Building the Package**

.. code-block:: bash

   $ colcon build
   ---
   Starting >>> demo_cpp_pkg
   Starting >>> demo_python_pkg
   Finished <<< demo_cpp_pkg [0.41s]
   Finished <<< demo_python_pkg [0.73s]

   Summary: 2 packages finished [0.85s]

In Listing 2-25, `colcon` is the tool used in ROS 2 to build packages. Using `colcon build` here will build all packages in the current and subdirectories. If the `build`, `install`, and `log` directories do not exist in the current directory, they will be automatically created, and the build intermediate files, results, and logs will be placed in the corresponding directories. After the build is complete, you can see the `cpp_node` executable in the `chapt2/install/demo_cpp_pkg/lib/demo_cpp_pkg/` directory. Next, you can run the executable by entering the two commands in Listing 2-26.

**Listing 2-26: Running the Node**

.. code-block:: bash

   $ source install/setup.bash
   $ ros2 run demo_cpp_pkg cpp_node
   ---
   [INFO] [1680684100.228612032] [cpp_node]: Hello C++ Node!

In Listing 2-26, the `source` command serves the same purpose as in the Python example in Section 2.2.1—it allows ROS 2 to find `demo_cpp_pkg` and its nodes. After running the command, you’ll see that the node has successfully started. At this point, we’ve completed writing a node in a C++ package. However, you should know that `colcon build` essentially calls `cmake` and `make` to compile the code.

