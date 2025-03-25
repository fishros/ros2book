2.5.2 Useful C++ New Features
=============================

Since the release of C++98 in 1998, the C++ standard has undergone multiple updates and revisions, each introducing new syntax, semantics, and features to the standard library. After 2000, C++ experienced several significant updates, including C++11 in 2011, which introduced features like smart pointers and lambda expressions, followed by C++14 in 2014, C++17 in 2017, and C++20 in 2020.

Compared to ROS 1, ROS 2 aligns more closely with modern robotics development requirements, as evident from its adoption of C++11 and higher standards. Many ROS 2 open-source libraries and frameworks also use C++ as the primary development language. Therefore, before diving deeper into ROS 2, let’s explore some useful C++ features.

In previous sections, when instantiating a ROS 2 node with `auto node = std::make_shared<rclcpp::Node>("cpp_node")`, we used two C++ features. Let’s start by learning about these.

1. Automatic Type Deduction with `auto`
------------------------------

The first feature is **automatic type deduction**, represented by the `auto` keyword. When instantiating a ROS 2 node, we used `auto`, which automatically deduces the variable type based on the value assigned to it. Let’s test this by writing some code. Create a `learn_auto.cpp` file in `chapt2_ws/src/demo_cpp_pkg/src/` and write the code in Listing 2-48.

**Listing 2-48: Testing Automatic Type Deduction in learn_auto.cpp**

.. code-block:: cpp

   #include <iostream>

   int main()
   {
       auto x = 5;
       auto y = 3.14;
       auto z = 'a';

       std::cout << x << std::endl;
       std::cout << y << std::endl;
       std::cout << z << std::endl;

       return 0;
   }

In Listing 2-48, we define three variables assigned integer, floating-point, and character values, all using the `auto` keyword. Then, we print these variables. Add the `learn_auto` node configuration to `CMakeLists.txt`, build it, and run it. The result is shown in Listing 2-49.

**Listing 2-49: Testing Automatic Type Deduction**

.. code-block:: bash

   $ ros2 run demo_cpp_pkg learn_auto
   ---
   5
   3.14
   a

From the result, you can see that variables defined with `auto` behave the same as normally defined variables. However, `auto` can significantly simplify code in certain situations.

2. Smart Pointers
------------------------------

The second feature is **smart pointers**, introduced in C++11. Smart pointers manage dynamically allocated memory, avoiding issues like memory leaks and null pointer dereferencing. C++11 provides three types of smart pointers: `std::unique_ptr`, `std::shared_ptr`, and `std::weak_ptr`. In our code, `std::make_shared` creates a `std::shared_ptr` smart pointer. Let’s focus on `std::shared_ptr`.

In C, pointers store the addresses of other variables, and smart pointers do the same. However, when dynamically allocated memory is no longer needed, you must manually call `free` to release it. Forgetting to do so or releasing it prematurely can cause memory leaks or null pointer issues. Smart pointers solve this by keeping track of how many pointers reference the same resource. When the reference count drops to zero, the memory is automatically released, preventing premature or forgotten releases.

Let’s test this by writing some code. Create a `learn_shared_ptr.cpp` file in `chapt2_ws/src/demo_cpp_pkg/src/` and write the code in Listing 2-50.

**Listing 2-50: Learning Smart Pointers**

.. code-block:: cpp

   #include <iostream>
   #include <memory>

   int main()
   {
       auto p1 = std::make_shared<std::string>("This is a str.");
       std::cout << "p1's reference count: " << p1.use_count() << ", memory address: " << p1.get() << std::endl;

       auto p2 = p1;
       std::cout << "p1's reference count: " << p1.use_count() << ", memory address: " << p1.get() << std::endl;
       std::cout << "p2's reference count: " << p2.use_count() << ", memory address: " << p2.get() << std::endl;

       p1.reset();
       std::cout << "p1's reference count: " << p1.use_count() << ", memory address: " << p1.get() << std::endl;
       std::cout << "p2's reference count: " << p2.use_count() << ", memory address: " << p2.get() << std::endl;
       std::cout << "p2's content: " << p2->c_str() << std::endl;
       return 0;
   }

Smart pointers are defined in the `<memory>` header under the `std` namespace, so we include `<memory>`. In the `main` function, we create a smart pointer `p1` pointing to a `std::string` using `std::make_shared`. We then print the reference count (`p1.use_count()`) and memory address (`p1.get()`) of the resource pointed to by `p1`. At this point, the resource is only referenced by `p1`, so the reference count should be 1.

Next, we share the resource with `p2`. Now, both `p1` and `p2` reference the resource, so the reference count should be 2, and the memory address remains the same.

Finally, we call `p1.reset()`, which resets `p1`. Now, `p1` no longer points to the resource, but `p2` still does. The reference count for `p1` becomes 0, while `p2`'s reference count becomes 1. The content of the resource pointed to by `p2` remains unchanged.

Add the `learn_shared_ptr` node configuration to `CMakeLists.txt`, build it, and run it. The result is shown in Listing 2-51.

**Listing 2-51: Testing Smart Pointers**

.. code-block:: bash

   $ ros2 run demo_cpp_pkg learn_shared_ptr
   ---
   p1's reference count: 1, memory address: 0x5621fcb6cec0
   p1's reference count: 2, memory address: 0x5621fcb6cec0
   p2's reference count: 2, memory address: 0x5621fcb6cec0
   p1's reference count: 0, memory address: 0
   p2's reference count: 1, memory address: 0x5621fcb6cec0
   p2's content: This is a str.

From the result in Listing 2-51, you can see that even though `p1` was reset, the resource is not released because `p2` still holds it. The resource’s value is still accessible. Imagine managing a resource with a smart shared pointer in a program. No matter how many functions the data is passed through, no resource copying occurs, significantly improving runtime efficiency. When all programs finish using the resource, it is automatically reclaimed, preventing memory leaks. This is why ROS 2 extensively uses smart pointers.

3. Lambda Expressions
------------------------------

You might have heard of anonymous functions. **Lambda expressions**, introduced in C++11, are a type of anonymous function. They have no name but can be called like regular functions. Lambda expressions have their own syntax, as shown in Listing 2-52.

**Listing 2-52: Lambda Expression Syntax**

.. code-block:: cpp

   [capture list](parameters) -> return_type { function body }

Here, the `capture list` captures external variables, `parameters` are the function parameters, `return_type` is the return type, and `function body` is the function’s implementation. Create a `learn_lambda.cpp` file in `chapt2_ws/src/demo_cpp_pkg/src/` and write the code in Listing 2-53.

**Listing 2-53: Using a Lambda Function to Calculate and Print the Sum of Two Numbers**

.. code-block:: cpp

   #include <iostream>
   #include <algorithm>

   int main()
   {
       auto add = [](int a, int b) -> int { return a + b; };
       int sum = add(3, 5);
       auto print_sum = [sum]()->void { std::cout << "3 + 5 = " << sum << std::endl; };
       print_sum();
       return 0;
   }

In Listing 2-53, we first define a lambda function `add` that takes two integers and returns their sum. The capture list is empty, and the function body is `return a + b`. We then call `add` to calculate `3 + 5` and store the result in `sum`. Next, we define another lambda function `print_sum`, capturing `sum` in the capture list. This allows us to directly print the value of `sum` in the function body. Finally, add the `learn_lambda` node configuration to `CMakeLists.txt`, build it, and run it. The result is shown in Listing 2-54.

**Listing 2-54: Running the learn_lambda Executable**

.. code-block:: bash

   $ ros2 run demo_cpp_pkg learn_lambda
   ---
   3 + 5 = 8

The result matches our expectations. You might not yet appreciate the benefits of lambda expressions, but you’ll grow to love them as you continue learning.

4. Function Wrapper `std::function`
------------------------------

`std::function`, introduced in C++11, is a general-purpose function wrapper. It can store any callable object (functions, function pointers, lambda expressions, etc.) and provide a unified calling interface. The concept might sound abstract, so let’s dive into some code. Create a `learn_function.cpp` file in `chapt2_ws/src/demo_cpp_pkg/src/` and write the code in Listing 2-55.

**Listing 2-55: Using Different Types of Functions to Create Function Wrappers**

.. code-block:: cpp

   #include <iostream>
   #include <functional>

   void save_with_free_fun(const std::string &file_name)
   {
       std::cout << "Called free function, saving: " << file_name << std::endl;
   }

   class FileSave
   {
   public:
       void save_with_member_fun(const std::string &file_name)
       {
           std::cout << "Called member function, saving: " << file_name << std::endl;
       };
   };

   int main()
   {
       FileSave file_save;
       auto save_with_lambda_fun = [](const std::string &file_name) -> void
       {
           std::cout << "Called lambda function, saving: " << file_name << std::endl;
       };
       // Store a free function in a function object
       std::function<void(const std::string &)> save1 = save_with_free_fun;
       // Store a lambda function in a function object
       std::function<void(const std::string &)> save2 = save_with_lambda_fun;
       // Store a member function in a function object
       std::function<void(const std::string &)> save3 = std::bind(&FileSave::save_with_member_fun, &file_save, std::placeholders::_1);
       // All functions can be called uniformly
       save1("file.txt");
       save2("file.txt");
       save3("file.txt");
       return 0;
   }

The code is a bit long, so let’s break it down. The `<functional>` header is included because it contains the function wrapper. We define a free function `save_with_free_fun` outside any class, which takes a file name as a parameter. Then, we define a `FileSave` class with a member function `save_with_member_fun` that also takes a file name as a parameter.

In the `main` function, we first instantiate a `FileSave` object `file_save` and create a lambda function `save_with_lambda_fun`. Next, we create three `std::function<void(const std::string &)>` objects using three different methods: direct assignment of a free function, assignment of a lambda function, and binding a member function using `std::bind`. Finally, we call the three wrapped functions.

Let’s focus on `std::bind`. It binds a member function to an object, turning it into a `std::function` object. Normally, you call a member function using the object and function name, like `file_save.save_with_member_fun`. Here, `std::bind` binds the member function `FileSave::save_with_member_fun` to the object `file_save` and uses `std::placeholders::_1` to reserve a spot for the function’s parameter.

Add the `learn_function` node configuration to `CMakeLists.txt`, build it, and run it. The result is shown in Listing 2-56.

**Listing 2-56: Running learn_function**

.. code-block:: bash

   $ ros2 run demo_cpp_pkg learn_function
   ---
   Called free function, saving: file.txt
   Called lambda function, saving: file.txt
   Called member function, saving: file.txt

By now, you’ve learned many new C++ features. With this foundation, you’ll find it much easier to dive into ROS 2 robotics development using C++. Next, we’ll explore the final topic needed for future development: **multithreading and callback functions**.