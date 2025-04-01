3.3.1 Publishing Velocity to Make the Turtle Draw a Circle
==========================================================

After the turtle simulator node starts running, it automatically generates a turtle. The simulator node subscribes to the topic named `/turtle1/cmd_vel` to receive control commands, and the message type of this topic is `geometry_msgs/msg/Twist`. While subscribing, the node also publishes its current position, with the message type of this topic being `turtlesim/msg/Pose`. To enable subscribing to and publishing the corresponding topics in the code, dependencies on `geometry_msgs` and `turtlesim` must be added when creating the package. Open the terminal, navigate to the directory `chapt3/topic_ws/src`, and enter the command in Code Listing 3-17 to create the `demo_cpp_topic` package.

**Listing 3-17 Create the `demo_cpp_topic` package and add the dependencies**

.. code-block:: bash

  $ ros2 pkg create demo_cpp_topic --build-type ament_cmake \
        --dependencies rclcpp geometry_msgs turtlesim \
        --license Apache-2.0
  ---
  going to create a new package
  package name: demo_cpp_topic
  destination directory: /home/fishros/chapt3/topic_ws/src
  package format: 3
  version: 0.0.0
  description: TODO: Package description
  maintainer: ['fishros <fishros@todo.todo>']
  licenses: ['Apache-2.0']
  build type: ament_cmake
  dependencies: ['rclcpp', 'geometry_msgs', 'turtlesim']
  ...

After creating the feature pack, create a `turtle_circle.cpp` file under `src/demo_cpp_topic/src`, and then write the code as shown in Listing 3-18 in the file.

**Listing 3-18 src/demo_cpp_topic/src/turtle_circle.cpp**

.. code-block:: cpp


   #include "rclcpp/rclcpp.hpp"
   #include "geometry_msgs/msg/twist.hpp"
   #include <chrono>
   using namespace std::chrono_literals;

   class TurtleCircle : public rclcpp::Node
   {
   private:
     rclcpp::TimerBase::SharedPtr timer_;
     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

   public:
     explicit TurtleCircle(const std::string& node_name) : Node(node_name)
     {
       publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
       timer_ = this->create_wall_timer(1000ms, std::bind(&TurtleCircle::timer_callback, this));
     }

   private:
     void timer_callback()
     {
       auto msg = geometry_msgs::msg::Twist();
       msg.linear.x = 1.0;
       msg.angular.z = 0.5;
       publisher_->publish(msg);
     }
   };

   int main(int argc, char *argv[])
   {
     rclcpp::init(argc, argv);
     auto node = std::make_shared<TurtleCircle>("turtle_square");
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
   }
In Code Listing 3-18, the ROS 2 client library `rclcpp` and the message interface `geometry_msgs/msg/twist.hpp` are first included. Then, the time library header is introduced, and the `using` declaration is used to enable time unit literals. Literals are a new feature in C++14, allowing direct use of numbers followed by `s` or `ms` to represent time, making the code more intuitive.

After importing the headers, a `TurtleCircle` class is defined, with two attributes added: a shared pointer to a timer (`timer_`) and a shared pointer to a topic publisher (`publisher_`). These attributes are then initialized in the constructor.

The `this->create_publisher` method is inherited from the parent class and is used to initialize the publisher. The `<>` syntax is a C++ template, and the enclosed `geometry_msgs::msg::Twist` specifies the topic's interface type. The first parameter of this method is the topic name, which must match the topic name subscribed to by the turtle for communication to work. The second parameter is `10`, which, as in Python, relates to ROS 2's Quality of Service (QoS) and is explained in detail in Chapter 10. Here, `10` represents the history queue length.

The `this->create_wall_timer` method also comes from the parent node and is used to initialize the timer. The first parameter is the call interval, set here to `1000ms`, meaning it will be triggered every second. The second parameter is the callback function, where the member method `timer_callback` is converted into a directly callable callback function using `std::bind`.

Inside the `timer_callback` method, a message object of type `geometry_msgs::msg::Twist` is first created. The linear velocity `x` in the forward direction is set to 1 meter per second, and the rotational angular velocity around the z-axis is set to 0.5 radians per second. At this point, the turtle's turning radius should be 1 divided by 0.5, equaling 2 meters.

Finally, in `CMakeLists.txt`, the `turtle_circle` node is added along with its dependencies. The main instructions are as shown in Code Listing 3-19.

**Listing 3-19 src/demo_cpp_topic/CMakeLists.txt**

.. code-block:: txt

   add_executable(turtle_circle src/turtle_circle.cpp)
   ament_target_dependencies(turtle_circle rclcpp geometry_msgs)

   install(TARGETS
     turtle_circle
     DESTINATION lib/${PROJECT_NAME}
   )
   ament_package()

Save and build the demo_cpp_topic package, run the turtle simulator, and then run the turtle_circle node. Observe the turtle simulator, and the result should be as shown in Figure 3-3.

.. figure:: figure3-3.png
    :alt: The turtle draws a circular trajectory.
    :align: center

    Figure 3-3 The turtle draws a circular trajectory.




