3.5.3 Learning to Use Git Ignore Files
--------------------------------------

Compared to adding directories or files one by one using the `git add` command, using `git add .` is much more convenient. However, in a project, there may be files or directories that you do not want to commit, and you do not want to manually exclude them one by one. This is where the Git ignore file comes in handy.

Create a new file named `.gitignore` in the `topic_practice_ws` directory, and then list the directories or files you want to ignore in this file. Here, we will add the directories generated during the ROS 2 build process. The final content of the file is shown in Code Listing 3-54.

.. code-block:: text

   **Code Listing 3-54: topic_practice_ws/.gitignore**

   build/
   install/
   log/

After creating the `.gitignore` file, commit it to the repository. It is a good practice to frequently commit your changes. Enter the commands in Code Listing 3-55 in the terminal.

.. code-block:: bash

   **Code Listing 3-55: Committing the .gitignore File**

   $ git add .gitignore
   $ git commit -m "Add Git ignore file"

.. code-block:: text

   ---
   [master 3071f21] Add Git ignore file
   3 files changed, 98 insertions(+)
   create mode 100644 .gitignore

After committing the `.gitignore` file, test whether it works by trying to add and commit files again. Enter the commands in Code Listing 3-56 in the terminal.

.. code-block:: bash

   **Code Listing 3-56: Testing if .gitignore Works**

   $ git add .
   $ git commit -m "Test commit temporary directories"

.. code-block:: text

   ---
   On branch master
   nothing to commit, working tree clean

The prompt indicates that the `install`, `build`, and `log` directories have been successfully ignored. In addition to ignoring specific directories and files, the `.gitignore` file can also use the wildcard ``*`` to ignore a category of files. For example, adding `*.log` to the `.gitignore` file will ignore all files with the `.log` extension.

That concludes our introduction to Git for now. Although we have only covered a few basic commands, you have already learned the fundamental usage of Git. We will continue to explore more Git features in the future. Now, let's summarize this chapter.