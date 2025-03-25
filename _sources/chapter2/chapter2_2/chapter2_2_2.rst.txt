2.2.2 Package Structure Analysis
================================

Open the `demo_python_pkg` folder in VS Code, and you’ll see the directory structure as shown in Figure 2-10.

.. figure:: figure2-10.png
    :alt: Python Package Structure
    :align: center

    Figure 2-10 Python Package Structure

At first glance, there are many files, and you might feel a bit overwhelmed. Don’t worry—I’ll explain the contents of Figure 2-10, and after that, it won’t seem so daunting.

- **demo_python_pkg**
  This directory’s name matches the package name by default. It is the directory where node code is placed and will be your main workspace for developing Python nodes. The `__init__.py` file in this folder is an identifier for a Python package. In other words, if a folder contains this file, it indicates that the folder is a Python package. This file is empty by default.

- **resource**
  This directory can hold some resources. In the package, this directory and the `demo_python_pkg` file under it are special and are mainly used to provide package identification. We don’t need to worry about or manually modify this file.

- **test**
  This is the folder for test code, used to place unit test files. Combined with the testing-related commands in the `colcon` build tool, it can perform unit tests on the code and generate reports. Unit testing is important in large-scale project development but is less relevant for now, so just be aware of it.

- **LICENSE**
  This file is the license for the package. You might recall the `--license Apache-2.0` parameter used when creating the package. The content of this file is the Apache-2.0 license agreement. When we open-source or share the package with others, the license helps protect intellectual property. There are many types of licenses, and Apache-2.0 is a flexible open-source license that is friendly to commercial use.

- **package.xml**
  This file is the manifest file for the package. Every ROS 2 package contains this file. It declares the package name, version number, maintainer, build type, license, and dependencies, among other information. In the previous section, before building the code, we added the `rclpy` dependency to this file. In fact, the build and run process would work without adding it, but I encourage you to declare the dependencies used in the code in the manifest. When sharing or porting the package, this file helps quickly identify the required dependencies. Additionally, ROS 2 can help manage dependencies during the build process, simplifying the build.

- **setup.cfg**
  This is a plain text file used to store configuration options for building the Python package. These configurations are read and processed during the build.

- **setup.py**
  This file is the build script for the Python package. It contains a `setup()` function that specifies how to build and install the Python package. When adding a node, you need to declare the executable name and the corresponding function in this file. We’ve already done this in the previous section, so I won’t elaborate further.

In addition to the files and folders mentioned above, you can also add files or folders based on your needs during actual development.