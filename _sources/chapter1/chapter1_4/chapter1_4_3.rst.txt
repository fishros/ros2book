1.4.3 Writing Python Programs in Linux
=================================

When we develop robots, the two most commonly used languages are C++ and Python. As shown in Figure 1-28, the programming language ranking chart, they are also among the top in the programming language rankings. So next, I will guide you through writing a "Hello World" program in Linux using Python and C++ in sequence.

.. figure:: figure1-28.png
    :alt: Programming Language Rankings

    Figure 1-28 Programming Language Rankings

In any terminal, enter `code` to open the VS Code main interface, as shown in Figure 1-29.

.. figure:: figure1-29.png
    :alt: VS Code Main Interface

    Figure 1-29 VS Code Main Interface

Next, in VS Code, click on **File > Open Folder** and navigate to the `chapt1` directory created in the home directory during Section 1.4.1. As shown in Figure 1-30, right-click on a blank area in the Explorer pane and select **New File**. Then, enter the file name `hello_world.py` and press Enter.

.. figure:: figure1-30.png
    :alt: VS Code Create New File

    Figure 1-30 VS Code Create New File

At this point, the file editing area will appear on the right side of VS Code, as shown in Figure 1-31. In this area, enter the content from Listing 1-23.

.. figure:: figure1-31.png
    :alt: VS Code Edit File

    Figure 1-31 VS Code Edit File

**Listing 1-23: Contents of ~/chapt1/hello_world.py**

.. code-block:: python

   print('Hello World!')

After entering the code, remember to save it using `Ctrl + S`. Beginners often forget to save their code, so you can enable the auto-save feature in VS Code by clicking **File > Auto Save**. Next, right-click on a blank area in the Explorer pane again and select **Open in Integrated Terminal**. The integrated terminal is essentially the same as the terminal we used earlier, but it is now integrated into VS Code.

In the terminal, enter the command in Listing 1-24 to run the code. The result is shown in Figure 1-32.

**Listing 1-24: Running the hello_world.py Script**

.. code-block:: bash

   $ python3 hello_world.py
   ---
   Hello World!

.. figure:: figure1-32.png
    :alt: Run Code in Terminal

    Figure 1-32 Run Code in Terminal

Well, have you noticed how convenient it is to write and run code in Ubuntu and VS Code?