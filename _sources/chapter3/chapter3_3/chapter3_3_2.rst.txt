3.3.2 Closed-loop Control Using Pose Subscription
=================================================

By publishing speed control commands to the topic `/turtle1/cmd_vel`, the turtle can be controlled to move. By subscribing to `/turtle1/pose`, the turtle's real-time position can be obtained. With both real-time position and speed control available, closed-loop control can be implemented to move the turtle to a specified position. Closed-loop control refers to adjusting the system's input by measuring and feeding back the output, thereby bringing the system's output closer to the desired value. In this section, we will continuously monitor the error between the turtle's current position and the target position, adjust the published commands in real time, and ultimately guide the turtle to the specified target point.

In `src/demo_cpp_topic/src`, create a new file named `turtle_control.cpp` and write the code as shown in Listing 3-20.

**Listing 3-20 turtle_control.cpp**

.. code-block:: cpp

    #include "geometry_msgs/msg/twist.hpp"
    #include "rclcpp/rclcpp.hpp"
    #include "turtlesim/msg/pose.hpp"

    class TurtleController : public rclcpp::Node {
     public:
      TurtleController() : Node("turtle_controller") {
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10);
        pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&TurtleController::on_pose_received_, this, std::placeholders::_1));
      }

     private:
      void on_pose_received_(const turtlesim::msg::Pose::SharedPtr pose) {
        // TODO: Calculate error based on received position and publish velocity command
      }

     private:
      rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
      double target_x_{1.0};  // Target X position (default: 1.0)
      double target_y_{1.0};  // Target Y position (default: 1.0)
      double k_{1.0};         // Proportional coefficient (output = error * k)
      double max_speed_{3.0}; // Maximum linear velocity (default: 3.0)
    };

    int main(int argc, char **argv) {
      rclcpp::init(argc, argv);
      auto node = std::make_shared<TurtleController>();
      rclcpp::spin(node);
      rclcpp::shutdown();
      return 0;
    }

In Listing 3-21, the ROS 2 related header files are first imported, where `turtlesim/msg/Pose` is the position topic message interface. This was previously seen in Section 3.1 when examining the simulator node details. The detailed definition of this message can be viewed using the command in Listing 3-21.


**Listing 3-21 View turtlesim/msg/Pose interface definition**

.. code-block:: cpp

    $ ros2 interface show turtlesim/msg/Pose
    ---
    float32 x
    float32 y
    float32 theta

    float32 linear_velocity
    float32 angular_velocity

Following the code listing 3-20, a `TurtleController` class is defined, which inherits from `Node`, and directly sets the node name to `turtle_controller` in the constructor. In the constructor, a topic publisher `velocity_publisher_` is first created. Then, the `create_subscription` method is called, with the topic message interface set to `turtlesim::msg::Pose` using template syntax `<>`. This method has three input parameters: the first parameter `"/turtle1/pose"` is the topic name, the second parameter `10` is the history queue length, and the third parameter is a callback function encapsulated via a member method. When data is received in the background, this callback function will be invoked for processing.

To react immediately upon receiving the current position each time, we place the core processing logic in the callback function. Next, let's complete the `on_pose_received_` method by filling in the code from Listing 3-22.


**Listing 3-22 View turtlesim/msg/Pose interface definition**

.. code-block:: cpp

    void on_pose_received_(const turtlesim::msg::Pose::SharedPtr pose) {
        auto message = geometry_msgs::msg::Twist();
        // 1. Record current position
        double current_x = pose->x;
        double current_y = pose->y;
        RCLCPP_INFO(this->get_logger(), "Current position:(x=%f,y=%f)", current_x, current_y);

        // 2. Calculate distance to target and angular difference from current turtle orientation
        double distance =
            std::sqrt((target_x_ - current_x) * (target_x_ - current_x) +
                      (target_y_ - current_y) * (target_y_ - current_y));
        double angle =
            std::atan2(target_y_ - current_y, target_x_ - current_x) - pose->theta;

        // 3. Control strategy: if distance > 0.1 continue moving; if angular difference > 0.2 rotate in place, otherwise move straight
        if (distance > 0.1) {
          if(fabs(angle)>0.2)
          {
            message.angular.z = fabs(angle);
          }else{
            // Calculate output velocity via proportional controller
            message.linear.x = k_ * distance;
          }
        }

        // 4. Limit maximum value and publish message
        if (message.linear.x > max_speed_) {
           message.linear.x = max_speed_;
        }
        velocity_publisher_->publish(message);
      }

The parameter of the callback function is a shared pointer to the received data, so the first step is to obtain the current position of the turtle via the pointer and record it. The second step involves calculating the distance and angle between the current position and the target position using the Euclidean distance formula. Here, the function prototype of `atan2` is `double atan2(double y, double x)`, which is used to compute the arctangent of `y` divided by `x`. Geometrically, this calculates the angle of the target position relative to the current position. The result is then subtracted from the current orientation to derive the `angle`.The third step is the core closed-loop control strategy: if the distance is greater than 0.1, the angular and linear velocities are calculated. Next, it checks whether the angle difference exceeds 0.2. If it does, an angular velocity command is sent to steer the turtle; otherwise, the linear velocity is computed based on the distance and a proportional coefficient. It’s evident that the greater the distance, the faster the linear velocity.The fourth step involves capping the maximum linear velocity before publishing the control message.

Add the instructions from Listing 3-23 to CMakeLists.txt to register the turtle_control node.


**Listing 3-23 Registering the node in CMakeLists.txt**

.. code-block:: cpp

    ...
    add_executable(turtle_control src/turtle_control.cpp)
    ament_target_dependencies(turtle_control rclcpp geometry_msgs turtlesim)

    install(TARGETS
      turtle_control
      turtle_circle
      DESTINATION lib/${PROJECT_NAME}
    )
    ...
    ament_package()

After completion, first run the turtle simulator, then compile and run the turtle_control node as shown in Listing 3-24.


**Listing 3-24 Running the turtle control node**

.. code-block:: cpp


    ros2 run demo_cpp_topic turtle_control
    ---
    [INFO] [1681554826.220945971] [turtle_controller]: Current position: (x=5.544445,y=5.544445)
    [INFO] [1681554826.237126893] [turtle_controller]: Current position: (x=5.544445,y=5.544445)
    ...
    [INFO] [1699337616.177492400] [turtle_controller]: Current position: (x=0.994040,y=1.098285)
    ...

When observing the turtle simulator, you can see the turtle moving to the target position in a graceful arc, as shown in Figure 3-4.

.. figure:: figure3-4.png
    :alt: TControl the turtle to move to the target point.
    :align: center

    Figure 3-4 Control the turtle to move to the target point.

Alright, at this point, you’ve learned how to subscribe to and publish data over topics in your code, and you’ve also picked up some closed-loop control along the way. Take a short break—next, we’ll work on a small project to put it all into practice.