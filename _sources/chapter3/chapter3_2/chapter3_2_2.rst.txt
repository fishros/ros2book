3.2.2 Subscribing to Novels and Speech Synthesis
===============================================

Before subscribing to the novel topic, let's first solve the speech synthesis issue. In Python, this can be achieved by installing third-party libraries. Open a terminal and enter the commands from Listing 3-14 sequentially.

**Listing 3-14: Installing the Speech Synthesis Engine**

.. code-block:: bash

    $ sudo apt install python3-pip -y
    $ sudo apt install espeak-ng -y
    $ pip3 install espeakng

The first command in Listing 3-14 uses apt to install pip3, Python 3's package management tool. The second command uses apt to install the espeak-ng speech synthesis engine. The final command uses pip3 to install the espeakng Python library for programmatic access. After installing the dependencies, we can write the node code.

Create a file named novel_sub_node.py under src/demo_python_topic/demo_python_topic with the content shown in Listing 3-15.

**Listing 3-15: src/demo_python_topic/demo_python_topic/novel_sub_node.py**

.. code-block:: python

    import rclpy
    from rclpy.node import Node
    from example_interfaces.msg import String
    import threading
    from queue import Queue
    import time
    import espeakng


    class NovelSubNode(Node):
        def __init__(self, node_name):
            super().__init__(node_name)
            self.novels_queue_ = Queue()
            self.novel_subscriber_ = self.create_subscription(
                String, 'novel', self.novel_callback, 10)
            self.speech_thread_ = threading.Thread(target=self.speak_thread)
            self.speech_thread_.start()

        def novel_callback(self, msg):
            self.novels_queue_.put(msg.data)

        def speak_thread(self):
            speaker = espeakng.Speaker()
            speaker.voice = 'zh'
            while rclpy.ok():
                if self.novels_queue_.qsize() > 0:
                    text = self.novels_queue_.get()
                    self.get_logger().info(f'Reading aloud: {text}')
                    speaker.say(text)
                    speaker.wait()
                else:
                    time.sleep(1)


    def main(args=None):
        rclpy.init(args=args)
        node = NovelSubNode("novel_read")
        rclpy.spin(node)
        rclpy.shutdown()

In the code above, the library import section is largely similar to that of a subscription node, with the only additions being the time library and the espeakng speech synthesis library.

The NovelSubNode implementation begins with its initialization method, where it first creates a queue to store received novel content. This queue is necessary because the speech synthesis speed cannot keep up with the message reception rate. The code then calls self.create_subscription(String, "novel", self.novel_callback, 10) to create a topic subscriber. Here, the first parameter String specifies the message type, the second parameter "novel" is the topic name, and the third parameter novel_callback is the callback function that gets automatically invoked with the received data when spin processes novel topic events. The callback function novel_callback places the received data at the end of the queue. The fourth parameter (10) configures the Quality of Service by setting the history message queue length.

After creating the subscriber, the code initializes a speech synthesis thread speech_thread_, which executes the speak_thread method. Within this method, it first creates an espeakng.Speaker object and configures it to use the Chinese voice ('zh'). The thread then continuously checks rclpy's status using rclpy.ok(). While active, it monitors the queue - when data is available, it retrieves the first item using novels_queue_.get(), synthesizes speech via speaker.say(), and waits for completion with speaker.wait(). If no data is available, the thread sleeps for 1 second using time.sleep(1).

Finally, the main function instantiates the NovelSubNode and executes spin. After registering novel_sub_node in setup.py and compiling, the node can be run as shown in Listing 3-16.

**Listing 3-16: Running the Novel Reading Node**

.. code-block:: bash

    $ ros2 run demo_python_topic novel_sub_node
    ---

    [INFO] [1681469282.747851102] [novel_read]: Reading aloud: "With hard work, you can definitely master ROS 2."

If you don't see any output, it may be because no topic is being published. You can either manually publish data via the command line or simply start the novel publishing node.

At this point, we've completed learning how to publish and subscribe to topics using Python, along with implementing speech synthesis in our code. Let's take a short break before moving on to learn how to work with topics in C++.