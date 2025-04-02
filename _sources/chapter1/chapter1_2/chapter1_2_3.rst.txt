1.2.3 Installing Ubuntu 22.04 in the Virtual Machine
===================================================

To install the system, we need a system image. Since accessing the official Ubuntu website can be unstable, we can download the image from a domestic mirror site, such as: http://mirrors.ustc.edu.cn/ubuntu-releases/22.04. Visit the URL and click on `ubuntu-22.04.x-desktop-amd64.iso` to start the download, where `x` represents the minor version code, which may vary depending on the time of access. After the download is complete, you will obtain a system image file with the `.iso` extension.

Next, return to VirtualBox and click the "New" button on the toolbar. You will see an interface as shown in Figure 1-6. Enter a name, select the folder where you want to place the virtual machine, and choose the `.iso` image file you just downloaded in the virtual optical disk section. Finally, remember to check the "Skip Automatic Installation" option, and click "Next" to proceed.

.. figure:: figure1-6.png
    :alt: VirtualBox Create New Virtual Machine Interface
    :align: center

    Figure 1-6 VirtualBox Create New Virtual Machine Interface

Next, you will see the interface for setting the virtual machine's memory and processor count, as shown in Figure 1-7. You can configure these settings based on your machine's specifications. Once the setup is complete, click "Next" to proceed.

.. figure:: figure1-7.png
    :alt: Virtual Machine Memory and Processor Count Settings Interface
    :align: center

    Figure 1-7 Virtual Machine Memory and Processor Count Settings Interface

Next, you will see the virtual hard disk setup interface as shown in Figure 1-8. Here, we allocate 120GB of space, as ROS 2 is a powerful platform and will likely require significant storage. Finally, click "Next" and then "Finish" to complete the virtual machine creation. Once created, select the virtual machine and click the start button shown in Figure 1-9 to launch it.

.. figure:: figure1-8.png
    :alt: Virtual Hard Disk Setup Interface
    :align: center

    Figure 1-8 Virtual Hard Disk Setup Interface

.. figure:: figure1-9.png
    :alt: Virtual Machine Start Button
    :align: center

    Figure 1-9 VVirtual Machine Start Button

On the first startup, you will see the selection interface shown in Figure 1-10. Use the keyboard arrow keys to select "Try or Install Ubuntu" and press Enter. After that, you will see the installer interface shown in Figure 1-11.

.. figure:: figure1-10.png
    :alt: First Boot Selection Interface
    :align: center

    Figure 1-10 First Boot Selection Interface

.. figure:: figure1-11.png
    :alt: Installer Interface
    :align: center

    Figure 1-11 Installer Interface

As shown in Figure 1-12, select your preferred language on the installer's language selection interface, and then click "Install Ubuntu." On the keyboard layout page, click "Continue" again to proceed to the interface shown in Figure 1-13. Here, choose "Minimal Installation" and uncheck the option to download updates during installation. Then, click "Continue" to proceed to the installation type page.

.. figure:: figure1-12.png
    :alt: Select Language
    :align: center

    Figure 1-12 Select Language

.. figure:: figure1-13.png
    :alt: Updates and Other Software Selection
    :align: center

    Figure 1-13 Updates and Other Software Selection

On the installation type interface, use the default erase option, and then click "Install Now." At this point, a confirmation window as shown in Figure 1-14 will pop up. Click "Continue" to proceed.

.. figure:: figure1-14.png
    :alt: Confirm Writing Changes to Disk Interface
    :align: center

    Figure 1-14 Confirm Writing Changes to Disk Interface

Next, on the location selection interface, simply choose your location and click "Continue." You will then proceed to the user setup interface shown in Figure 1-15. Set your username and password. Since you will frequently need to enter the password later, it can be kept short to avoid input errors. After that, click "Continue" to begin the formal installation process.

.. figure:: figure1-15.png
    :alt: Username and Password Setup Interface
    :align: center

    Figure 1-15 Username and Password Setup Interface

Next, simply wait for the installation to complete. If you encounter a section that takes a long time to install, you can click "Skip" to speed up the installation process.

Once you see the prompt indicating that the installation is complete, click "Restart Now" to enter the system login interface shown in Figure 1-16. Use the mouse to click on your username, enter the password you just set, and press Enter to log into the system. Upon the first startup, there will be a prompt page, which you can close by clicking the button in the upper-right corner.

.. figure:: figure1-16.png
    :alt: Installation Interface
    :align: center

    Figure 1-16 Installation Interface

At this point, we have completed the installation of the Ubuntu 22.04 virtual machine. Of course, during the learning process, you may occasionally need to uninstall it due to various issues. To do so, right-click on the corresponding virtual machine in the VirtualBox virtual machine list and select "Remove."