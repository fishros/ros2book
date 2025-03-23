2.4 Best Practices for Multi-Package Workspaces
==============================================

Through the previous sections, you’ve successfully created ROS 2 packages and learned how to write and run nodes within them. However, you might have noticed some minor issues. For example, when there are multiple packages in the current directory, using the build command will build all packages, even if you only need to build one. Additionally, packages and temporary files generated during compilation are in the same directory, which can become messy as the number of packages increases. How can we solve these problems? In this section, we’ll learn the best practices for organizing multiple packages.

A complete robot often consists of multiple functional modules, so it’s necessary to combine multiple packages. ROS 2 developers have established the concept of a **Workspace**—a workspace. Open `chapt2/` in VS Code, open the integrated terminal, navigate to the `chapt2` directory, and enter the command in Listing 2-27.

**Listing 2-27: Creating a Workspace**

.. code-block:: bash

   $ mkdir -p chapt2_ws/src

In the command above, the `-p` parameter means recursive creation. After creating `chapt2_ws`, a `src` folder is created within it, resulting in a workspace. You might ask, isn’t this just a regular folder? Why is it called a workspace? The reason is that a workspace is essentially a concept and convention. During development, we place all packages in the `src` directory and run `colcon` for building in the same directory as `src`. The build results (`build`, `install`) and logs (`log`) are kept at the same level as `src`. Next, navigate to the `chapt2` directory in the terminal and enter the commands in Listing 2-28.

**Listing 2-28: Moving Packages to chapt2_ws/src/**

.. code-block:: bash

   $ mv demo_cpp_pkg/   chapt2_ws/src/
   $ mv demo_python_pkg/   chapt2_ws/src/
   $ rm -rf build/ install/ log/

The first two commands in Listing 2-28 move the existing packages to `chapt2_ws/src/`, and the third command deletes the directories generated during previous builds. Next, navigate to the `chapt2/chapt2_ws/` directory in the terminal and enter the command in Listing 2-29 to build the packages.

**Listing 2-29: Building the Packages**

.. code-block:: bash

   $ colcon build
   ---
   Starting >>> demo_cpp_pkg
   Starting >>> demo_python_pkg
   Finished <<< demo_python_pkg [0.94s]
   Finished <<< demo_cpp_pkg [4.19s]

   Summary: 2 packages finished [4.34s]

The `colcon build` command lives up to expectations, scanning and building all packages in the current workspace. However, if you want to build a specific package, such as `demo_cpp_pkg`, you can use the `--packages-select` command followed by the package name. The test command is shown in Listing 2-30.

**Listing 2-30: Building a Specific Package**

.. code-block:: bash

   $ colcon build --packages-select demo_cpp_pkg
   ---
   Starting >>> demo_cpp_pkg
   Finished <<< demo_cpp_pkg [0.19s]

   Summary: 1 package finished [0.29s]

When using `colcon build`, all packages will start building simultaneously, provided the CPU allows it. As shown in the build log in Listing 2-30, both packages start building at the same time, with `demo_python_pkg` finishing first, followed by `demo_cpp_pkg`.

However, sometimes dependencies exist between packages in the same workspace. For example, `demo_python_pkg` might depend on the build results of `demo_cpp_pkg`. In this case, you need to build `demo_cpp_pkg` first, and then build `demo_python_pkg`. To achieve this, simply declare the dependency in the package’s manifest file. Open `demo_python_pkg/package.xml` and add a dependency on `demo_cpp_pkg`. The updated content is shown in Listing 2-31.

**Listing 2-31: Adding a Dependency on a Package in the Workspace**

.. code-block:: xml

   <?xml version="1.0"?>
   …
     <depend>rclpy</depend>
     <depend>demo_cpp_pkg</depend>
     <test_depend>ament_copyright</test_depend>
   …
   </package>

After saving, enter the build command in Listing 2-32 again.

**Listing 2-32: Controlling Build Order Through Dependencies**

.. code-block:: bash

   $ colcon build
   ---
   Starting >>> demo_cpp_pkg
   Finished <<< demo_cpp_pkg [0.18s]
   Starting >>> demo_python_pkg
   Finished <<< demo_python_pkg [0.56s]

   Summary: 2 packages finished [0.85s]

As you can see, after running the command, `demo_cpp_pkg` is built first, and then `demo_python_pkg` starts building.

With this, our learning about ROS 2 packages and workspaces comes to an end. The next focus will be on the code itself. So, take a short break, and let’s dive into some foundational knowledge needed for ROS 2 programming.