1.3.2 Controlling the Turtle with the Keyboard
=============================================

Do not close the turtle simulator window. Open a new terminal again using the shortcut and enter the command shown in Listing 1-7.

**Listing 1-7: Launching the Turtle Keyboard Control Program**

.. code-block:: bash

   $ ros2 run turtlesim turtle_teleop_key
   ---
   Reading from keyboard
   ---------------------------
   Use arrow keys to move the turtle.
   Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
   'Q' to quit.

The command in Listing 1-7 means: **Use ROS 2 to run the `turtle_teleop_key` keyboard control node from the `turtlesim` package.** After running it, you can use the arrow keys to move the little turtle as prompted.

To allow the keyboard control node to capture keyboard input, click on the terminal where the keyboard control node is running with your mouse. Then, press the arrow keys to observe the turtle's movement path, as shown in Figure 1-20.

.. figure:: figure1-20.png
    :alt: Turtle Motion Path
    :align: center

    Figure 1-20 Turtle Motion Path