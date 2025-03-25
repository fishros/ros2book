2.5.1 Object-Oriented Programming
=================================

You can find standardized definitions of object-oriented programming (OOP) in many tutorials, but these definitions are often conceptual and easier to understand for those already familiar with the topic. For beginners, they can be confusing. Therefore, I’ll use simple language and practical examples to explain what OOP is.

Unlike procedural languages like C, object-oriented languages allow you to create **classes**, which are encapsulations of entities. For example, humans, phones, robots, and any other entities can be encapsulated into classes. A class can have its own **attributes** and **methods**. For instance, humans have attributes like height and age, while phones have attributes like brand and memory size. These attributes typically describe the class. Methods represent the behavior of the class, such as humans eating and sleeping, or phones turning on and off.

Through this encapsulation, you can instantiate an object of a class when needed. For example, when someone needs to eat, you can create a person object named "Zhang San" and call its `eat` method to perform the action. In addition to instantiating a class into a concrete object, classes can also be **inherited**, which we’ll discuss later. For now, let’s try creating a class.

2.5.1.1 Python Example
------------------------------

Open `chapt2_ws/` in VS Code, then create a `person_node.py` file in the `src/demo_python_pkg/demo_python_pkg` directory. Add the code shown in Listing 2-33.

**Listing 2-33: Defining a Simple Python Class**

.. code-block:: python

   class PersonNode:
       def __init__(self) -> None:
           pass

This is an empty class declaration. In Python, we use the `class` keyword to declare a class and add an empty `__init__` method. The `__init__` method is a special method in Python classes that is called when an object of the class is created. When you need to add attributes to the class, you typically pass initial values through the parameters of the `__init__` method. Now, let’s add some attributes and methods to this class, as shown in Listing 2-34.

**Listing 2-34: Adding Attributes and Methods to the Python Class**

.. code-block:: python

   class PersonNode:
       def __init__(self, name: str, age: int) -> None:
           print('PersonNode\'s __init__ method was called')
           self.age = age
           self.name = name

       def eat(self, food_name: str):
           print(f'My name is {self.name}, I am {self.age} years old, and I am eating {food_name}')

In the `__init__` method, we created the `age` and `name` attributes, requiring a string for the name and an integer for the age. We also added a print statement to check if the method is called. Then, we defined an `eat` method that takes a string `food_name` as a parameter. Note that the first parameter of all Python class methods is `self`, which refers to the instance itself.

Now that the `PersonNode` class is defined, let’s instantiate it and call its methods. Add the instantiation code at the end of the `person_node.py` file, as shown in Listing 2-35.

**Listing 2-35: Defining the main Function to Instantiate PersonNode**

.. code-block:: python

   class PersonNode:
       ...

   def main():
       node = PersonNode('Zhang San', 18)
       node.eat('Fish-flavored Shredded Pork')

Here, we pass the name and age as parameters to `PersonNode`, instantiate an object of the `PersonNode` class named `node`, and then call its `eat` method. Let’s try compiling and running the code. First, register the current node in `setup.py`. After completing this, the `entry_points` section in `setup.py` should look like Listing 2-36.

**Listing 2-36: setup.py**

.. code-block:: python

   entry_points={
           'console_scripts': [
               'python_node = demo_python_pkg.python_node:main',
               'person_node = demo_python_pkg.person_node:main',
           ],
       },

Next, start the build process. Open the terminal and enter the command in Listing 2-37.

**Listing 2-37: Building a Single Package**

.. code-block:: bash

   $ colcon build --packages-select demo_python_pkg
   ---
   Starting >>> demo_python_pkg
   Finished <<< demo_python_pkg [0.60s]

   Summary: 1 package finished [0.70s]

Then, as shown in Listing 2-38, run the `person_node` node directly.

**Listing 2-38: Running the person_node Node**

.. code-block:: bash

   $ source install/setup.bash
   $ ros2 run demo_python_pkg person_node
   ---
   PersonNode's __init__ method was called
   My name is Zhang San, I am 18 years old, and I am eating Fish-flavored Shredded Pork

The result matches our expectations. After learning how to encapsulate attributes and methods into a class, let’s explore another important feature of OOP: **inheritance**.

Suppose we want to define a `WriterNode` class for writers. Writers have their own books, so we can add a `book` attribute to the `WriterNode` class. However, writers are also people, so they have names and ages and can eat. If we add `age`, `name`, and `eat` to `WriterNode`, it would be redundant. Instead, we can have `WriterNode` inherit the attributes and methods of `PersonNode`. Let’s implement this. Create a `writer_node.py` file in the `src/demo_python_pkg/demo_python_pkg` directory and add the code in Listing 2-39.

**Listing 2-39: Creating the WriterNode Class Inheriting from PersonNode**

.. code-block:: python

   from demo_python_pkg.person_node import PersonNode

   class WriterNode(PersonNode):
       def __init__(self, book: str) -> None:
           print('WriterNode\'s __init__ method was called')
           self.book = book

   def main():
       node = WriterNode('On Fast Imprisonment')
       node.eat('Fish-flavored Shredded Pork')

In Listing 2-39, we first import the `PersonNode` class from `person_node.py`. We define `WriterNode` and specify that it inherits from `PersonNode` by adding `PersonNode` in parentheses after the class name. Then, we add the `book` attribute in the `__init__` method. Next, we instantiate a `WriterNode` object `node`, assign values to its attributes, and call the `eat` method. Now, add the `writer_node` node to `setup.py` and rebuild. You should see the result shown in Listing 2-40.

**Listing 2-40: Running the writer_node Node**

.. code-block:: bash

   $ ros2 run demo_python_pkg writer_node
   ---
   WriterNode's __init__ method was called
   Traceback (most recent call last):
     File "/home/fishros/chapt2/chapt2_ws/install/demo_python_pkg/lib/demo_python_pkg/writer_node", line 33, in <module>
       sys.exit(load_entry_point('demo-python-pkg==0.0.0', 'console_scripts', 'writer_node')())
     File "/home/fishros/chapt2/chapt2_ws/install/demo_python_pkg/lib/python3.10/site-packages/demo_python_pkg/writer_node.py", line 10, in main
       node.eat('Fish-flavored Shredded Pork')
     File "/home/fishros/chapt2/chapt2_ws/install/demo_python_pkg/lib/python3.10/site-packages/demo_python_pkg/person_node.py", line 12, in eat
       print(f'Age {self.age}, name {self.name} is eating {food_name}')
   AttributeError: 'WriterNode' object has no attribute 'age'
   [ros2run]: Process exited with failure 1

Don’t panic when encountering errors—I’ll teach you how to handle them. The error message here, from top to bottom, first shows the call stack of the error. We called the `node.eat()` method, and the error occurred inside the `eat` method when it tried to print. The error message is `AttributeError: 'WriterNode' object has no attribute 'age'`. Even though we made `WriterNode` inherit from `PersonNode`, why does `WriterNode` not have the `age` attribute? From the first print statement, you can see that the `__init__` method of `PersonNode` was not called. We can fix this by using `super()` to call the parent class’s `__init__` method. Modify `writer_node.py` as shown in Listing 2-41.

**Listing 2-41: Adding a Call to the Parent Class’s __init__ Method**

.. code-block:: python

   …
   class WriterNode(PersonNode):
       def __init__(self, name: str, age: int, book: str) -> None:
           super().__init__(name, age)
           print('WriterNode\'s __init__ method was called')
           self.book = book
   …

After saving, rebuild the code to copy it to the `install` directory. Run `writer_node` again, and the result is shown in Listing 2-42.

**Listing 2-42: Running the Updated writer_node Node**

.. code-block:: bash

   $ ros2 run demo_python_pkg writer_node
   PersonNode's __init__ method was called
   WriterNode's __init__ method was called
   My name is Zhang San, I am 18 years old, and I am eating Fish-flavored Shredded Pork

From the result in Listing 2-42, you can see that `WriterNode` has successfully inherited the attributes and methods of `PersonNode`.

After learning about Python classes and inheritance, let’s turn `PersonNode` and `WriterNode` into actual ROS 2 nodes. Recall that when writing the first Python node, we instantiated the `Node` class. If we make `PersonNode` inherit from `Node`, it will have all the attributes and methods of the `Node` class, becoming a true ROS 2 node. Let’s try this by modifying `person_node.py` as shown in Listing 2-43.

**Listing 2-43: PersonNode Inheriting from ROS 2 Node**

.. code-block:: python

   import rclpy
   from rclpy.node import Node

   class PersonNode(Node):
       def __init__(self, node_name: str, name: str, age: int) -> None:
           super().__init__(node_name)
           self.age = age
           self.name = name

       def eat(self, food_name: str):
           self.get_logger().info(f'My name is {self.name}, I am {self.age} years old, and I am eating {food_name}')

   def main():
       rclpy.init()
       node = PersonNode('person_node', 'Zhang San', 18)
       node.eat('Fish-flavored Shredded Pork')
       rclpy.spin(node)
       rclpy.shutdown()

Here, we first import the `rclpy` library and the `Node` class. Then, we make `PersonNode` inherit from `Node` and require a node name to be passed when calling the parent class’s `__init__` method. Finally, we change the `eat` method’s print statement to use ROS 2’s logger. Rebuild and run the code, and the result is shown in Listing 2-44.

**Listing 2-44: Running the Updated writer_node Node**

.. code-block:: bash

   $ ros2 run demo_python_pkg writer_node
   ---
   [INFO] [1680891408.833994560] [person_node]: My name is Zhang San, I am 18 years old, and I am eating Fish-flavored Shredded Pork

In Listing 2-43, we used the `self.get_logger()` method, which belongs to the `Node` class, and successfully printed the message. This shows that `PersonNode` has successfully inherited the `Node` class.

Since `WriterNode` inherits from `PersonNode`, it should also have the attributes and methods of the `Node` class. Try converting it into a ROS 2 node and running it as a homework assignment.

After learning so many OOP concepts, you might feel overwhelmed or find the process tedious. OOP requires continuous practice in future learning and work to truly appreciate its benefits. Take a break, and in the next section, we’ll explore OOP in C++.

2.5.1.2 C++ Example
------------------------------

After learning the basic concepts of OOP and implementing them in Python, you’ll find this section on C++ OOP easier to grasp. Like Python, C++ is a high-level language with similar OOP features, though the syntax differs. Let’s create a C++ version of the `PersonNode` node using the concepts from the previous section.

Create a `person_node.cpp` file in the `chapt2_ws/src/demo_cpp_pkg/src` directory and write the code in Listing 2-45.

**Listing 2-45: Writing a C++ PersonNode Inheriting from Node**

.. code-block:: cpp

   #include <string>
   #include "rclcpp/rclcpp.hpp"

   class PersonNode : public rclcpp::Node
   {
   private:
     std::string name_;
     int age_;

   public:
     PersonNode(const std::string &node_name,
                const std::string &name,
                const int &age) : Node(node_name)
     {
       this->name_ = name;
       this->age_ = age;
     };

     void eat(const std::string &food_name)
     {
       RCLCPP_INFO(this->get_logger(), "I am %s, I am %d years old, and I am eating %s",
                   name_.c_str(), age_, food_name.c_str());
     };
   };

   int main(int argc, char **argv)
   {
     rclcpp::init(argc, argv);
     auto node = std::make_shared<PersonNode>("cpp_node", "Zhang San", 18);
     node->eat("Fish-flavored ROS");
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
   }

Starting from the top, the code includes the `string` and `rclcpp/rclcpp.hpp` headers. The `string` header is included because node names and person names are represented as strings.

Next, we define the `PersonNode` class using the `class` keyword, making it inherit from `rclcpp::Node`. Inside the class, we first define the `private` section, which includes the `name_` and `age_` attributes.

In the `public` section, we define the constructor `PersonNode`, which takes the node name, name, and age as parameters. Note that the parameters are passed as `const` references (`const std::string &`). The `&` indicates passing by reference, which avoids unnecessary data copying and improves efficiency. The `const` ensures the variables are read-only, preventing accidental modifications and enhancing code safety. The `: Node(node_name)` syntax calls the parent class’s constructor, passing the node name parameter. This is similar to Python but with different syntax.

Inside the constructor, we use `this`, a pointer to the current object, to assign values to `name_` and `age_`. Then, we implement the `eat` method, which takes a `const` reference to the food name. Inside the method, we use ROS 2’s logging module to print the data. Since `RCLCPP_INFO` uses C-style formatted printing, `name_` and `food_name` need to be converted to C-style strings using `c_str()`.

Finally, in the `main` function, we use `std::make_shared` to create a `PersonNode` object, passing the node name, name, and age as parameters. We then call the `eat` method and ROS 2’s related methods. Next, modify `CMakeLists.txt` to add the `person_node` node. The complete `CMakeLists.txt` is shown in Listing 2-46.

**Listing 2-46: Adding the person_node Node**

.. code-block:: cmake

   ...
   # find dependencies
   find_package(ament_cmake REQUIRED)
   ...
   add_executable(person_node src/person_node.cpp)
   ament_target_dependencies(person_node rclcpp)

   install(TARGETS
     cpp_node person_node
     DESTINATION lib/${PROJECT_NAME}
   )
   ...
   ament_package()

Next, enter the following commands to build and run the node. The commands and results are shown in Listing 2-47.

**Listing 2-47: Building and Running the person_node Node**

.. code-block:: bash

   $ colcon build --packages-select demo_cpp_pkg
   $ source install/setup.bash
   $ ros2 run demo_cpp_pkg person_node
   ---
   [INFO] [1680904724.328635258] [cpp_node]: I am Zhang San, I am 18 years old, and I am eating Fish-flavored ROS

At this point, you should feel accomplished because you’ve successfully written a C++ node class using OOP. Over the years, C++ has introduced many new features, such as `std::make_shared` for smart pointers, which are widely used in ROS 2. Take a short break, and in the next section, we’ll explore some of the new C++ features useful for ROS 2 development.