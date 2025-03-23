1.4.4 Writing C++ Programs in Linux
==================================

With the experience of writing Python code, writing C++ programs is also straightforward. Create a new file `hello_world.cpp` in the `chapt1` directory and enter the content from Listing 1-25. Then, save the code and run it.

**Listing 1-25: hello_world.cpp**

.. code-block:: cpp

   #include <iostream>
   int main()
   {
       std::cout << "Hello World !" << std::endl;
       return 0;
   }

You can directly use the command-line tool `g++` to compile the code in Ubuntu. Open the integrated terminal, navigate to the `chapt1` directory, and enter the command in Listing 1-26 to compile the `hello_world.cpp` code using `g++`.

**Listing 1-26: Compiling hello_world.cpp Using g++**

.. code-block:: bash

   $ g++ hello_world.cpp
   $ ls
   ---
   a.out hello_world.cpp hello_world.py

After compilation, you’ll notice that a new file `a.out` has been created in the directory. The file name is displayed in green, indicating that it is an executable file. Use the command in Listing 1-27 to execute this file.

**Listing 1-27: Executing a.out**

.. code-block:: bash

   $ ./a.out
   ---
   Hello World !

Using `g++` to compile simple code files is very convenient. However, for complex code with various dependencies, the CMake tool is more suitable. Create a new file `CMakeLists.txt` in the `chapt1` directory and enter the content from Listing 1-28.

**Listing 1-28: CMakeLists.txt File**

.. code-block:: cmake

   cmake_minimum_required (VERSION 3.8)
   project (HelloWorld)
   add_executable(learn_cmake hello_world.cpp)

The `CMakeLists.txt` file contains three lines of instructions. The first line specifies the minimum required version of CMake for building the project. The second line declares the project name. The third line adds an executable file, where `learn_cmake` is the name of the executable file, and `hello_world.cpp` is the path to the source file relative to `CMakeLists.txt`.

After writing and saving the file, navigate to the `chapt1` directory in the terminal and enter the command in Listing 1-29 to convert `CMakeLists.txt` into a `Makefile`.

**Listing 1-29: Generating Makefile Using cmake**

.. code-block:: bash

   $ cmake .
   ---
   -- The C compiler identification is GNU 11.4.0
   -- The CXX compiler identification is GNU 11.4.0
   -- Detecting C compiler ABI info
   -- Detecting C compiler ABI info - done
   -- Check for working C compiler: /usr/bin/cc - skipped
   -- Detecting C compile features
   -- Detecting C compile features - done
   -- Detecting CXX compiler ABI info
   -- Detecting CXX compiler ABI info - done
   -- Check for working CXX compiler: /usr/bin/c++ - skipped
   -- Detecting CXX compile features
   -- Detecting CXX compile features - done
   -- Configuring done
   -- Generating done
   -- Build files have been written to: /home/fishros/chapt1

The `cmake` command is used to build the `CMakeLists.txt` file. The `.` parameter indicates that `CMakeLists.txt` is located in the same directory as the terminal. After the build is complete, the result files will be generated in the current directory, including the `Makefile`. The `Makefile` can be read by the `make` command to compile the code. Continue by entering the command in Listing 1-30 in the terminal.

**Listing 1-30: Compiling Using the make Command**

.. code-block:: bash

   $ make
   ---
   [ 50%] Building CXX object CMakeFiles/learn_cmake.dir/hello_world.cpp.o
   [100%] Linking CXX executable learn_cmake
   [100%] Built target learn_cmake

The `make` command invokes the compiler to convert the code into an executable file. After running the `make` command, check the files in the `chapt1` directory, and you will see the generated `learn_cmake` executable file. Use the command in Listing 1-31 to execute it.

**Listing 1-31: Executing learn_cmake**

.. code-block:: bash

   $ ./learn_cmake
   ---
   Hello World !