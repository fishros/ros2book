3.4.3 System Information Acquisition and Publication
==================================================

In the src directory, use the command from Listing 3-31 to create the status_publisher package, and add the message interface status_interfaces along with the client library rclpy as its dependencies.

**Listing 3-31: Create the status_publisher package**

.. code-block:: bash

   ros2 pkg create status_publisher --build-type ament_python --dependencies rclpy status_interfaces --license Apache-2.0

Next, create the file `sys_status_pub.py` in the corresponding package directory, and then write the code as shown in Code Listing 3-32.

**Listing 3-32: status_publisher/status_publisher/sys_status_pub.py**

.. code-block:: python

   import rclpy
   from rclpy.node import Node
   from status_interfaces.msg import SystemStatus
   import psutil
   import platform

   class SysStatusPub(Node):
       def __init__(self, node_name):
           super().__init__(node_name)
           self.status_publisher_ = self.create_publisher(
               SystemStatus, 'sys_status', 10)
           self.timer = self.create_timer(1, self.timer_callback)

       def timer_callback(self):
           cpu_percent = psutil.cpu_percent()
           memory_info = psutil.virtual_memory()
           net_io_counters = psutil.net_io_counters()

           msg = SystemStatus()
           msg.stamp = self.get_clock().now().to_msg()
           msg.host_name = platform.node()
           msg.cpu_percent = cpu_percent
           msg.memory_percent = memory_info.percent
           msg.memory_total = memory_info.total / 1024 / 1024
           msg.memory_available = memory_info.available / 1024 / 1024
           msg.net_sent = net_io_counters.bytes_sent / 1024 / 1024
           msg.net_recv = net_io_counters.bytes_recv / 1024 / 1024

           self.get_logger().info(f'Publishing: {str(msg)}')
           self.status_publisher_.publish(msg)

   def main():
       rclpy.init()
       node = SysStatusPub('sys_status_pub')
       rclpy.spin(node)
       rclpy.shutdown()


In Code Listing 3-32, the code first imports the `rclpy` and `Node` modules, then imports the `SystemStatus` class from `status_interfaces.msg`, and finally imports the `psutil` and `platform` modules. Using `psutil`, we can obtain the system's CPU, memory, and network information, while the `platform` module allows us to get the current hostname.

Next, the `SysStatusPub` class is defined, which inherits from `Node`. In the initialization function, a publisher `status_publisher_` and a timer `timer` are created. The timer calls the `timer_callback` method every 1 second to publish data. In the `timer_callback` method, the current node clock time is first obtained using the built-in method `get_clock().now()` inherited from `Node`, and then converted into a `builtin_interfaces.msg.Time` message object using `to_msg()` and assigned to `stamp`. Subsequently, the CPU usage, memory information, and network information are obtained using `psutil`. Next, a `SystemStatus` message is constructed, and the hostname is obtained using `platform.node()`. The remaining data are assigned values accordingly. Since the default data units are in Bytes, dividing by 1024 twice converts the units to MB. Finally, the data is printed as a string, and the `status_publisher_` is called to publish the data.

The `main` function contains basic operations, which are not elaborated here. Next, the `sys_status_pub` node is registered in the `setup.py` file, and then the command in Code Listing 3-33 is used to compile and run the code.


**Listing 3-33: Run the system status publisher node**

.. code-block:: bash

   $ ros2 run status_publisher sys_status_pub
   ---
   [INFO] [1681661308.321525494] [sys_status_pub]: 发布:status_interfaces.msg.SystemStatus(stamp=builtin_interfaces.msg.Time(sec=1681661308, nanosec=315946639), host_name='fishros-VirtualBox', cpu_percent=3.0, memory_percent=59.6, memory_total=3923.5078125, memory_available=1583.6796875, net_sent=2.044133186340332, net_recv=1.3831653594970703)

In addition to viewing the data in the terminal, you can also check it using command-line tools. Open a new terminal and enter the two commands listed in Code Listing 3-34 one after the other.


**Listing 3-34: Use the command line to print the data from the /sys_status topic.**

.. code-block:: bash

   source install/setup.bash
   ros2 topic echo /sys_status

   stamp:
     sec: 1681662249
     nanosec: 315808820
   host_name: fishros-VirtualBox
   cpu_percent: 16.799999237060547
   memory_percent: 60.900001525878906
   memory_total: 3923.5078125
   memory_available: 1532.8125
   net_sent: 2.4008235931396484
   net_recv: 1.6205368041992188

At this point, we have completed the system status retrieval and publishing part. Next, let's work on creating an interface to visualize the data.