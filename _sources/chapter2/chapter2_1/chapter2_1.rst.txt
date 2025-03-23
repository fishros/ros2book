2.1 Writing Your First Node
==========================

In Chapter 1, both the turtle simulator and the keyboard controller we ran generated corresponding ROS 2 nodes. The simulator node subscribes to topics from the keyboard control node to achieve the transmission of control commands. Therefore, you might think that a ROS 2 node is simply an executable program that can subscribe to or publish topics. As smart as you are, that’s indeed true, but you might still be underestimating the role of nodes. In addition to subscribing to and publishing topics, nodes can also use services, configure parameters, and execute actions, among other things.

As the saying goes, "A thousand learnings are not as good as one look, and a thousand looks are not as good as one practice." Next, we’ll write our first ROS 2 node using Python and C++ respectively.

 .. toctree::
    :maxdepth: 3

    chapter2_1_1