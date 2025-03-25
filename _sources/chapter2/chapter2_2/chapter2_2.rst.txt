2.2 Organizing Python Nodes Using Packages
=========================================

Through the previous section, you’ve successfully written your first Python node and your first C++ node. ROS 2 provides a better way to organize nodes: **packages**. Packages are tools in ROS 2 used to organize and manage nodes. After writing code within a package, you only need to configure a few instructions to use the build commands provided by ROS 2 to compile and install the nodes, making development more convenient. Besides facilitating development, packages also allow you to place functionally related nodes under the same package, making it easier to share and use them. For example, in Section 1.3, the turtle simulator and keyboard control nodes belong to different nodes within the same package.

Since different programming languages have different build methods, ROS 2 provides different types of packages for different development languages. Let’s first learn how to place Python nodes into a package.



.. toctree::
    :maxdepth: 3

    chapter2_2_1
    chapter2_2_2
