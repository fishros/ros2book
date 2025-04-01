3.2.1 Publishing Novels via Topics
=================================

Create a chapt3 folder in the home directory and open it with VS Code. Then create a workspace topic_ws in this folder, followed by a src directory under topic_ws. Open a terminal, navigate to src, and execute the command in Code Listing 3-8 to create the demo_python_topic package.

**Listing 3-8: Create an ament_python package with dependencies**


.. code-block:: bash

   $ ros2 pkg create demo_python_topic --build-type ament_python --dependencies rclpy example_interfaces --license Apache-2.0
   ---
   going to create a new package
   package name: demo_python_topic
   destination directory: /home/fishros/chapt3/topic_ws/src
   package format: 3
   version: 0.0.0
   description: TODO: Package description
   maintainer: ['fishros <fishros@todo.todo>']
   licenses: ['Apache-2.0']
   build type: ament_python
   dependencies: ['rclpy', 'std_msgs']
   ...

In code listing 3-8, the command to create the functional package is slightly different from before. By adding `--dependencies rclpy example_interfaces`, it includes the dependencies `rclpy` and `example_interfaces`, where `example_interfaces` is the message interface to be used next. At this point, if you open the package manifest file `package.xml`, you will notice that the dependency items for `rclpy` and `example_interfaces` have been automatically added.

After creating the package, create a novel_pub_node.py file in src/demo_python_topic/demo_python_topic and write the code shown in Code Listing 3-9.

**Listing 3-9: src/demo_python_topic/demo_python_topic/novel_pub_node.py**


.. code-block:: python

   import rclpy
   from rclpy.node import Node
   import requests
   from example_interfaces.msg import String
   from queue import Queue

   class NovelPubNode(Node):
       def __init__(self, node_name):
           super().__init__(node_name)
           self.novels_queue_ = Queue() # Create queue for novel content
           # Create publisher for novel topic
           self.novel_publisher_ = self.create_publisher(String, 'novel', 10)
           self.timer_ = self.create_timer(5, self.timer_callback) # Create timer

       def download_novel(self, url):
           response = requests.get(url)
           response.encoding = 'utf-8'
           self.get_logger().info(f'Download complete: {url}')
           for line in response.text.splitlines(): # Split by lines into queue
               self.novels_queue_.put(line)

       def timer_callback(self):
           if self.novels_queue_.qsize() > 0: # If queue has data, publish a line
               msg = String() # Instantiate message
               msg.data = self.novels_queue_.get() # Assign value
               self.novel_publisher_.publish(msg) # Publish message
               self.get_logger().info(f'Published novel line: {msg.data}')

   def main():
       rclpy.init()
       node = NovelPubNode('novel_pub')
       node.download_novel('http://localhost:8000/novel1.txt')
       rclpy.spin(node)
       rclpy.shutdown()

Code Listing 3-9 first imports rclpy libraries and the requests library for HTTP downloads. Since novels consist of text, it imports the String interface from example_interfaces. The interface definition can be viewed via command line as shown in Code Listing 3-10.

**Listing 3-10: View example_interfaces/msg/String interface definition**


.. code-block:: bash

   $ ros2 interface show example_interfaces/msg/String
   ---
   # This is an example message of using a primitive datatype, string.
   # If you want to test with this that's fine, but if you are deploying
   # it into a system you should create a semantically meaningful message type.
   # If you want to embed it in another message, use the primitive data type instead.
   string data

From the results in code listing 3-10, we can see that this interface contains a data field of type `string` named `data`, which we can use to represent text.

After importing the necessary libraries in code listing 3-9, a `NovelPubNode` node is defined. In its `__init__` method, a queue is created to store the paragraphs of the novel to be published. Then, `self.create_publisher(String, "novel", 10)` is called to create a topic publisher. This method takes three parameters:
1. The first, `String`, is the interface type of the topic, using the `String` imported earlier from `example_interfaces.msg`.
2. The second parameter is the name of the topic.
3. The third parameter is related to ROS 2's Quality of Service (QoS) policy, which will be discussed in detail in Chapter 10 of this book. For now, you only need to know that `10` represents the queue length for storing historical messages.

After creating the publisher, `self.create_timer(5, self.timer_callback)` is called to create a timer, where the first parameter is the period (in seconds), and the second is the callback function. Once set up, the node will call the `timer_callback` function every 5 seconds.

The node provides a download_novel method externally, which takes the URL of the novel as a parameter. This method uses the requests library to download the novel, splits it into lines, and then places them into the queue by calling the put method of novels_queue_.

Finally, there is the timer_callback method. If it detects that the novel queue has content (length > 0), it creates a String-type message object (msg), retrieves a piece of data from the queue, assigns it to the data attribute of msg, and then publishes the message using the publisher.

In the `main` function, the node is instantiated. First, `download_novel` is called to fetch the novel, followed by `spin` to keep the node running.

In `setup.py`, `novel_pub_node` is registered. After running the Python local server introduced in Section 2.5.3 and adding a few more lines to the novel, rebuild the project and run `novel_pub_node`. The command and its output are shown in code listing 3-11.

**Listing 3-11: Run novel_pub_node**


.. code-block:: bash

   $ ros2 run demo_python_topic novel_pub_node
   ---
   [INFO] [1681459937.288372889] [novel_pub]: Download complete: http://localhost:8000/novel1.txt
   [INFO] [1681459939.501703634] [novel_pub]: Published novel line: Chapter 1: The youth embarks on the path of cultivation, exiled for his Zhuxian power.
   [INFO] [1681459944.507272228] [novel_pub]: Published novel line: In an ancient village lived a young man.

TIt appears that the novel has been successfully downloaded and published, but to confirm whether it has actually been published, you need to test it via the command line. Open a new terminal and enter the command in Listing 3-12.

**Listing 3-12: View topic list**


.. code-block:: bash

   $ ros2 topic list -v
   ---
   Published topics:
    * /novel [std_msgs/msg/String] 1 publisher
   ...

The -v flag shows detailed information including the /novel topic. Use Code Listing 3-13 to view real-time topic content.

**Listing 3-13: Print novel topic data via command line**


.. code-block:: bash

   $ ros2 topic echo /novel
   ---
   data: This ancient village was nestled in a quiet valley, surrounded by lush forests and tranquil streams.
   ---
   data: The young man's name was Li Ming, who had lived in this village for sixteen years.

While reading novels in the terminal is interesting, having the computer read them aloud would be better. Next, we'll attempt to subscribe to the novel topic and synthesize speech.