1.4.5 Linux Basics: Environment Variables
========================================

Environment variables in Linux are global variables used to store system and user-level configuration information. For example, the ROS version information for the current terminal is stored in the `ROS_DISTRO` environment variable. Use the command in Listing 1-32 to view the value of this environment variable.

**Listing 1-32: Viewing the ROS Version Configured in the Current Terminal**

.. code-block:: bash

   $ echo $ROS_DISTRO
   ---
   humble

Earlier, when we used `ros2 run` to execute the `turtlesim_node` from the `turtlesim` package, `ros2 run` first reads the value of the `AMENT_PREFIX_PATH` environment variable and then looks for the package and executable files in the `lib` directory under that path. You can use the command in Listing 1-33 to view the value of this environment variable.

**Listing 1-33: Viewing the AMENT_PREFIX_PATH Environment Variable**

.. code-block:: bash

   $ echo $AMENT_PREFIX_PATH
   ---
   /opt/ros/humble

To search for `turtlesim` under `$AMENT_PREFIX_PATH/lib`, use the command and result shown in Listing 1-34.

**Listing 1-34: Locating the turtlesim Package**

.. code-block:: bash

   $ ls $AMENT_PREFIX_PATH/lib | grep turtlesim
   ---
   turtlesim
   ...

From the result, you can see that the `turtlesim` directory indeed exists. To view all contents of this directory, including the `turtlesim_node` executable, use the command in Listing 1-35.

**Listing 1-35: Viewing All Contents in the turtlesim Directory**

.. code-block:: bash

   $ ls $AMENT_PREFIX_PATH/lib/turtlesim
   ---
   draw_square  mimic  turtlesim_node  turtle_teleop_key

You can try directly constructing the path to the executable using the environment variable and running it. The command and result are shown in Listing 1-36.

**Listing 1-36: Directly Running the turtlesim_node Executable**

.. code-block:: bash

   $ $AMENT_PREFIX_PATH/lib/turtlesim/turtlesim_node
   ---
   [INFO] [1698488208.170362529] [turtlesim]: Starting turtlesim with node name /turtlesim
   [INFO] [1698488208.179445208] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]

You can use the `export` command to set environment variables in Linux. Let’s modify the `AMENT_PREFIX_PATH` environment variable to an incorrect path and try running `turtlesim_node` again. You’ll notice that it cannot be found. The test command and result are shown in Listing 1-37.

**Listing 1-37: Testing Environment Variable Settings**

.. code-block:: bash

   $ export AMENT_PREFIX_PATH=/opt/ros/
   $ ros2 run turtlesim turtlesim_node
   ---
   Package 'turtlesim' not found

Without realizing it, we’ve already covered all the basic operations for Ubuntu. Now that you’ve learned enough, let’s summarize and review what we’ve covered.