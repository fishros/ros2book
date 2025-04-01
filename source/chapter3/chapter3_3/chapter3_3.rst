3.3 Topic Publishing and Subscribing in C++
==========================================


In practical robotics development projects, C++ is often used for motion control. In this section, we will use C++ and the Turtle simulator to first publish velocity commands via topics to make the turtle draw a circle. Then, we will subscribe to the turtle's current position via topics and adjust the control commands based on the difference between the current position and the target position, achieving closed-loop control of the turtle's position.

.. toctree::
    :maxdepth: 3

    chapter3_3_1
    chapter3_3_2