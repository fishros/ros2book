2.5.3 Multithreading and Callback Functions
==========================================

If you’re accustomed to procedural programming, multithreading might seem fascinating. The charm of multithreading lies in its ability to run programs in parallel. For example, you can use multithreading to download multiple novels simultaneously instead of waiting for one to finish before starting the next. Additionally, another key focus of this section is **callback functions**.

2.5.3.1 Python Example
------------------------------

We pass function A as a parameter to function B and call function A within function B through this parameter, enabling the processing of execution results. For instance, after a download completes, a callback function can provide feedback on the result. Let’s implement a Python example of downloading novels using multithreading. Create a `learn_thread.py` file in the `src/demo_python_pkg/demo_python_pkg` directory and add the code in Listing 2-57.

**Listing 2-57: Downloading Novels Locally**

.. code-block:: python

   import threading
   import requests

   class Download:
       def download(self, url, callback):
           print(f'Thread: {threading.get_ident()} started downloading: {url}')
           response = requests.get(url)
           response.encoding = 'utf-8'
           callback(url, response.text)

       def start_download(self, url, callback):
           thread = threading.Thread(target=self.download, args=(url, callback))
           thread.start()

   def download_finish_callback(url, result):
       print(f'{url} download completed, total: {len(result)} characters, content: {result[:5]}...')

   def main():
       d = Download()
       d.start_download('http://localhost:8000/novel1.txt', download_finish_callback)
       d.start_download('http://localhost:8000/novel2.txt', download_finish_callback)
       d.start_download('http://localhost:8000/novel3.txt', download_finish_callback)

In Listing 2-57, we first import the Python threading library `threading` and the HTTP request library `requests`. If you encounter import errors, you can install `python3-requests` using `apt`. Then, we define a `Download` class with two methods: `download` and `start_download`.

The `download` method takes a URL and a callback function as parameters. It prints the current thread ID and the URL being downloaded, then uses `requests` to fetch the data. Finally, it passes the URL and data text to the callback function.

The `start_download` method also takes a URL and a callback function as parameters. It creates a new thread to run the `download` function, passing the URL and callback as arguments, and starts the thread.

After defining the `Download` class, we create a `download_finish_callback` function to handle the downloaded data. In the `main` function, we instantiate a `Download` object and call `start_download` three times to download the novels, using `download_finish_callback` as the callback.

Before testing the code, let’s prepare the novels and a local HTTP server. Open a terminal, navigate to the `chapt2_ws` directory, and use the commands in Listing 2-58 to create three novel chapters and start a local HTTP server.

**Listing 2-58: Creating Three Novel Files and Running the Server**

.. code-block:: bash

   $ echo "Chapter 1: The boy embarks on the path of cultivation, exiled due to the power of the Slaying Immortal." > novel1.txt
   $ echo "Chapter 2: Learning cultivation, making friends, and understanding responsibility." > novel2.txt
   $ echo "Chapter 3: Zhang Jiajie returns to the village, resists evil, and becomes a guardian." > novel3.txt
   $ python3 -m http.server
   ---
   Serving HTTP on 0.0.0.0 port 8000 (http://0.0.0.0:8000/) ...

Add the `learn_thread` node to `setup.py`, build it, and run it. The result is shown in Listing 2-59.

**Listing 2-59: Multithreaded Novel Download**

.. code-block:: bash

   $ ros2 run demo_python_pkg learn_thread
   ---
   Thread: 140385711027776 started downloading: http://localhost:8000/novel1.txt
   Thread: 140385702635072 started downloading: http://localhost:8000/novel2.txt
   Thread: 140385694242368 started downloading: http://localhost:8000/novel3.txt
   http://localhost:8000/novel2.txt download completed, total: 20 characters, content: Chapter 2...
   http://localhost:8000/novel1.txt download completed, total: 22 characters, content: Chapter 1...
   http://localhost:8000/novel3.txt download completed, total: 23 characters, content: Chapter 3...

From the result, you can see that three threads were started for downloading. The shortest novel, `novel2.txt`, finished first, while `novel3.txt` finished last. The order isn’t fixed, as the files are small and download quickly. While multithreading has its benefits, too many threads can strain system scheduling. In ROS 2, data processing and scheduling are handled in a single thread by default.

2.5.3.2 C++ Example
------------------------------

Like Python, C++ can also run programs in parallel using multithreading, though the syntax and interfaces differ. Let’s dive into the code. Before writing the code, download the C++ HTTP request library `cpp-httplib`, which only requires including a header file. Open the integrated terminal, navigate to `chapt2_ws/src/demo_cpp_pkg/include`, and use the command in Listing 2-60 to download it via `git`.

**Listing 2-60: Downloading the Open-Source Library Using git**

.. code-block:: bash

   $ git clone https://gitee.com/ohhuo/cpp-httplib.git
   ---
   Cloning into 'cpp-httplib'...
   remote: Enumerating objects: 4527, done.
   remote: Counting objects: 100% (4527/4527), done.
   remote: Compressing objects: 100% (1422/1422), done.
   remote: Total 4527 (delta 3057), reused 4527 (delta 3057), pack-reused 0
   Receiving objects: 100% (4527/4527), 2.27 MiB | 805.00 KiB/s, done.
   Resolving deltas: 100% (3057/3057), done.

After downloading, add the `include_directories(include)` directive to `CMakeLists.txt` to specify the `include` folder as the header file directory. The final `CMakeLists.txt` is shown in Listing 2-61.

**Listing 2-61: Adding Dependencies Using include_directories**

.. code-block:: cmake

   ...
   find_package(rclcpp REQUIRED)
   include_directories(include)
   ...
   ament_package()

After completing the above steps, create a `learn_thread.cpp` file in `chapt2_ws/src/demo_cpp_pkg/src` and write the code in Listing 2-62.

**Listing 2-62: learn_thread.cpp**

.. code-block:: cpp

   #include <iostream>
   #include <thread>
   #include <chrono>
   #include <functional>
   #include <cpp-httplib/httplib.h>

   class Download
   {
   public:
       void download(const std::string &host, const std::string &path, const std::function<void(const std::string &, const std::string &)> &callback)
       {
           std::cout << "Thread ID: " << std::this_thread::get_id() << std::endl;
           httplib::Client client(host);
           auto response = client.Get(path);
           if (response && response->status == 200)
           {
               callback(path, response->body);
           }
       }

       void start_download(const std::string &host, const std::string &path, const std::function<void(const std::string &, const std::string &)> &callback)
       {
           auto download_fun = std::bind(&Download::download, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
           std::thread download_thread(download_fun, host, path, callback);
           download_thread.detach();
       }
   };

   int main()
   {
       Download download;
       auto download_finish_callback = [](const std::string &path, const std::string &result) -> void
       {
           std::cout << "Download completed: " << path << " Total: " << result.length() << " characters, content: " << result.substr(0, 16) << std::endl;
       };

       download.start_download("http://localhost:8000", "/novel1.txt", download_finish_callback);
       download.start_download("http://localhost:8000", "/novel2.txt", download_finish_callback);
       download.start_download("http://localhost:8000", "/novel3.txt", download_finish_callback);
       std::this_thread::sleep_for(std::chrono::milliseconds(1000 * 10));
       return 0;
   }

In the code above, we include the threading-related header `thread`, the time-related header `chrono`, the function wrapper header `functional`, and the `cpp-httplib/httplib.h` header for downloading. Then, we declare the `Download` class with two methods: `download` and `start_download`.

The `download` method takes three parameters: the host address, the path (essentially splitting the full URL into two parts), and a callback function to handle the result after a successful request.

The `start_download` method has the same parameters as `download`. Inside the method, we use `std::bind` to turn the member function `download` into a `std::function` object. We then create a `thread` object, passing the function, host address, path, and callback function. Unlike Python, C++ threads start running immediately upon creation. The `download_thread.detach()` method detaches the thread from the current process, allowing it to run in the background.

Finally, in the `main` function, we create an instance of the `Download` class, define a callback function using a lambda, and call `start_download` three times to download the files. We use `std::this_thread::sleep_for` to delay the main thread for 10 seconds, ensuring all downloads have enough time to complete.

Add the `learn_thread` node configuration to `CMakeLists.txt`, build the package, and run the code using the command in Listing 2-63.

**Listing 2-63: Running the learn_thread Node**

.. code-block:: bash

   $ ros2 run demo_cpp_pkg learn_thread
   ---
   Thread ID: 140551882073664
   Thread ID: 140551873680960
   Thread ID: 140551798126144
   Download completed: /novel3.txt Total: 65 characters, content: Chapter 3: Zhang...
   Download completed: /novel1.txt Total: 62 characters, content: Chapter 1: The...
   Download completed: /novel2.txt Total: 56 characters, content: Chapter 2: Lea...

From the result, you can see that three threads were started to download the files. However, due to differences in how C++ and Python count characters, the character counts differ.

That’s it for the foundational ROS 2 programming knowledge. Once you’ve mastered the concepts in this section, you’ll have no trouble understanding the code in the future.