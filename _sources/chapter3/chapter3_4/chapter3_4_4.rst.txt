3.4.4 Using Qt in a Package
===========================

Qt is a cross-platform software design and development tool. Many tools in ROS 2 use Qt as the interface design tool. For example, both the turtle simulator and the visualization tool rqt (ROS Qt-based Gui Toolkit) are implemented using Qt. Therefore, we will also use Qt to complete the interface display functionality. Let's first learn how to use Qt in a package.

Open a terminal and enter the command from Code Listing 3-35 to create a new package called `status_display` and add `status_interfaces` as a dependency.


**Listing 3-35: Create the `status_display` package.**

.. code-block:: bash

   $ ros2 pkg create status_display --build-type ament_cmake --dependencies rclcpp status_interfaces --license Apache-2.0

Next, create a file named `hello_qt.cpp` in the `status_display/src` directory of this package and write the code from Code Listing 3-36.


**Listing 3-36: A Simple Display Interface**

.. code-block:: cpp

   #include <QApplication>
   #include <QLabel>
   #include <QString>

   int main(int argc, char* argv[]) {
     QApplication app(argc, argv);
     QLabel* label = new QLabel();
     QString message = QString::fromStdString("Hello Qt!");
     label->setText(message);
     label->show();
     app.exec();
     return 0;
   }

In Code Listing 3-36, three header files are included. `<QApplication>` provides the Qt application class `QApplication`, `QLabel` is a component in Qt used for displaying text, which we will use to display data. `QString` is a string class in Qt used for storing strings.

In the main function, a `QApplication` object `app` is created. Then, a pointer to a `QLabel` object is created and dynamically allocated using `new`. The `QString::fromStdString` method is used to create a `QString` object `message` from a string. The `setText` method of the label is then called to set its display content to `message`. After setting it, `label->show()` is used to display the text component. Finally, `app.exec()` is called to start the event loop. Note that `app.exec()` is similar to the `spin` method in ROS 2, as both continuously loop to handle events and block the program from continuing execution.

After writing the code, modify the `CMakeLists.txt` file to add the `hello_qt` node and include the necessary Qt dependencies. The final content of `CMakeLists.txt` is shown in Code Listing 3-37.

**Listing 3-37: topic_practices_ws/src/status_display/CMakeLists.txt**

.. code-block:: cmake

   cmake_minimum_required(VERSION 3.8)
   ...
   find_package(status_interfaces REQUIRED)
   find_package(Qt5 REQUIRED COMPONENTS Widgets)

   add_executable(hello_qt src/hello_qt.cpp)
   target_link_libraries(hello_qt Qt5::Widgets)

   install(TARGETS hello_qt
     DESTINATION lib/${PROJECT_NAME})
   ...
   ament_package()

Since Qt is composed of multiple components, when using `find_package`, we specifically look for the Widgets component under Qt5. Note that Qt5 is a third-party library unrelated to ROS 2, so we do not use `ament_target_dependencies` to add dependencies. Instead, we use `target_link_libraries` and specify the component as Widgets.

After completing the code writing and configuration, enter the commands from Code Listing 3-38 in the terminal to build the package and run the node.

**Listing 3-38: Run `hello_qt`**

.. code-block:: bash

   $ colcon build --packages-select status_display
   $ source install/setup.bash
   $ ros2 run status_display hello_qt

When you see the interface shown in Figure 3-6, your first Qt project is complete. While the enthusiasm is still hot, let's learn how to use Qt in combination with ROS 2 to display messages.

.. figure:: figure3-6.png
    :alt: Qt-based Interface
    :align: center

    Figure 3-6 Qt-based Interface