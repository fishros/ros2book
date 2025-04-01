3.5.1 Creating a Code Repository
--------------------------------

In Section 2.5.5, we briefly used Git to download a C++ networking library to our local machine. However, downloading code is just the most basic function of Git. Now, let's learn how to create a Git repository locally.

Before creating a repository, we need to configure Git with some settings. The first settings to configure are your username and email address. When you commit code, Git will include this information in the commit. This is useful in collaborative projects to distinguish which code was submitted by whom. Open any terminal and enter the commands in Code Listing 3-43 to configure these settings.

**Listing 3-43: Configure Git user and email**

.. code-block:: bash

   $ git config --global user.name "Fish" # Your username
   $ git config --global user.email "fish@fishros.com" # Configure your email

After configuring the user information, we will configure the default branch name. Use the command in Code Listing 3-44 to set the default branch name to "master". We will revisit the concept of branches later.

**Listing 3-44: Configure the default branch.**

.. code-block:: bash

   $ git config --global init.defaultBranch master

Once all configurations are complete, you can use the command in Code Listing 3-45 to view all Git configurations and confirm whether the corresponding items have been set correctly.

**Listing 3-45: View Git configuration**

.. code-block:: bash
   $ git config -l

   ---
   user.name=Fish
   user.email=fish@fishros.com
   init.defaultbranch=master
   core.repositoryformatversion=0
   core.filemode=true
   core.bare=false
   core.logallrefupdates=true

Next, create a local code repository. Navigate to the `chapt3/topic_practice_ws` directory and enter the command in Code Listing 3-46 to create the repository.

**Listing 3-46: Initialize the repository**

.. code-block:: bash

   $ git init

   ---
   Initialized empty Git repository in /home/fishros/chapt3/topic_practice_ws/.git/

You will see that the terminal indicates that an empty repository has been initialized in the specified directory. In Linux, files and directories starting with a `.` are hidden. You can view them using `ls -a`, as shown in Code Listing 3-47.


**Listing 3-47: View all files (including hidden)**

.. code-block:: bash

   $ ls -a


   ---
   .  ..  build  .git  install  log  src  .vscode

When you commit code, all changes will be saved in the `.git` directory. Similarly, if you want to delete the repository, you can simply remove the directory using `rm -rf .git`.