1.4.1 Basic Linux Terminal Operations
=====================================

In Linux systems, the terminal is the most frequently used interface, so our learning will primarily revolve around it. First, let’s understand how to perform file-related operations in the terminal.

Use the shortcut `Ctrl + Alt + T` to open a new terminal, and then enter the command in Listing 1-9 to view the current directory of the terminal.

**Listing 1-9: Command to View the Current Terminal Directory**

.. code-block:: bash

   $ pwd  # View the current directory of the terminal
   ---
   /home/fishros

The `pwd` command is used to view the current directory of the terminal. The content after the `#` symbol is a comment. As you can see, the result of the `pwd` command is `/home/fishros`, which is the default directory when opening a terminal using the shortcut. You might wonder why there are no drive letters like "C:"—this is because in Linux, the root directory of the entire file system is represented by a forward slash `/`, and all other directories and files are relative to the root directory.

Next, enter the command in Listing 1-10 to navigate to the root directory and check if the current directory changes. The `cd` command is used to switch the current directory of the terminal. After entering, use `pwd` to confirm that the current directory has changed to `/`.

**Listing 1-10: Command to Switch to the Root Directory**

.. code-block:: bash

   $ cd /  # Switch to the root directory
   $ pwd
   ---
   /

Then, enter the command in Listing 1-11 to view the files in the current directory.

**Listing 1-11: Command to View Files in the Current Directory**

.. code-block:: bash

   $ ls  # View files in the current directory
   ---
   bin   dev  home  lib    lib64   lost+found  mnt  proc  run   snap  sys  usr
   boot  etc  init  lib32  libx32  media       opt  root  sbin  srv   tmp  var

When opening a terminal using the shortcut, it defaults to the home directory of the current user. In this example, the username is `fishros`, so the home directory is `/home/fishros`. Since the home directory is frequently used, Linux allows the use of the `~` symbol to represent the home directory. To return to the home directory, simply enter the command in Listing 1-12. Let’s go back to the home directory and see what files are there.

**Listing 1-12: Command to View Home Directory Contents**

.. code-block:: bash

   $ cd ~
   $ ls
   ---
   Public Templates Videos Pictures Documents Downloads Music Desktop snap

After learning how to switch directories and view file lists, let’s learn how to create, edit, and delete a file in the home directory. The commands and results are shown in Listing 1-13.

**Listing 1-13: Creating Folders and Files**

.. code-block:: bash

   $ cd ~                       # Navigate to the home directory
   $ mkdir chapt1               # Create a folder named chapt1 in the home directory
   $ cd chapt1                  # Navigate into the chapt1 folder
   $ touch hello_world.txt      # Create an empty file
   $ pwd                        # View the current path
   ---
   /home/fishros/chapt1
   $ ls                         # View all files in the chapt1 directory
   ---
   hello_world.txt

We used the `mkdir` command to create a folder named `chapt1`, then used the `cd` command to enter this folder. Next, we created the `hello_world.txt` file, and finally used the `pwd` and `ls` commands to view the current directory and files.

Now, let’s use a tool to edit the `hello_world.txt` file. Enter the command in Listing 1-14 to open the window shown in Figure 1-23.

**Listing 1-14: Editing a File with nano**

.. code-block:: bash

   $ nano hello_world.txt

.. figure:: figure1-23.png
    :alt: Editing Interface

    Figure 1-23 Editing Interface

`nano` is a text editing tool that comes pre-installed with Ubuntu. After entering `hello ros 2 !`, use `Ctrl + O` followed by Enter to write the content to the file, and then use `Ctrl + X` to exit the editor.

Next, we can use the `cat` command to view the file and the `rm` command to delete the file. The commands and results are shown in Listing 1-15.

**Listing 1-15: Viewing File Content and Deleting a File**

.. code-block:: bash

   $ cat hello_world.txt
   ---
   hello ros 2 !
   $ rm hello_world.txt

Linux terminal commands are very extensive, and you might feel overwhelmed by the number of commands and their usage. Here’s a handy tip: you can append `--help` to any command to view its help documentation. This will display all the usage information for the command. Let’s test this with two commands.

The first is the `rm` command we just learned, as shown in Listing 1-16. The second is the `ros2 run` command introduced in Section 1.3.1, as shown in Listing 1-17.

**Listing 1-16: Help for the `rm` Command**

.. code-block:: bash

   $ rm --help
   ---
   Usage: rm [OPTION]... [FILE]...
   Remove (unlink) one or more <FILE>s.
   ...

**Listing 1-17: Help for the `ros2 run` Command**

.. code-block:: bash

   $ ros2 run --help
   ---
   usage: ros2 run [-h] [--prefix PREFIX] package_name executable_name ...
   Run a package-specific executable.
   positional arguments:
     package_name     Name of the ROS package
     executable_name  Name of the executable
     argv             Pass arbitrary arguments to the executable
   options:
     -h, --help       show this help message and exit
     --prefix PREFIX  Prefix command, which should go before the executable.
                      Command must be wrapped in quotes if it contains spaces
                      (e.g. --prefix 'gdb -ex run --args').

With this method of checking command help, you no longer need to worry about forgetting how to use a command!