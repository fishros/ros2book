3.5.2 Learning to Commit Code
-----------------------------

Now that we have a repository, let's proceed to commit code. Committing code involves two main commands: `add` and `commit`. The `add` command is used to stage modified files, while the `commit` command actually commits the staged changes to the local Git repository.

For example, if we want to stage the file `src/status_interfaces/package.xml`, we can use the command in Code Listing 3-48.

.. code-block:: bash

   $ git add src/status_interfaces/package.xml

Staging one file at a time can be slow. To stage an entire directory, such as all files under `src`, we can use the command in Code Listing 3-49.

.. code-block:: bash

   $ git add src

If there are many directories in the project, even staging one directory at a time can be inefficient. By adding a ``.`` after the `add` command, we can stage all files in the current directory. Open a terminal and run the command in Code Listing 3-50.

.. code-block:: bash

   $ git add .

After running the `add` command, if there are files in the staging area, you can commit them directly. However, I do not recommend doing so immediately. In our project, the`install`, `build`, and `log` directories are generated during compilation and are not part of the source code. Using `add .` will stage all files, including these directories. Committing directly would include them as well. To avoid this, we can use another command to unstage all files. This command is `reset`. Enter the command in Code Listing 3-51 in the terminal.

.. code-block:: bash

   $ git reset

Then, stage only the `src` directory and commit the changes by entering the commands in Code Listing 3-52.

.. code-block:: bash

   $ git add src
   $ git commit -m "Completed status publishing and display functionality"

.. code-block:: text

   ---
   [master (root-commit) 243169b] Completed status publishing and display functionality
   19 files changed, 968 insertions(+)
   create mode 100644 src/status_display/CMakeLists.txt
   create mode 100644 src/status_display/LICENSE
   create mode 100644 src/status_display/package.xml
   ...

Note that we added a description message using the `-m` option in the `commit` command. It is essential to provide a description when committing; otherwise, the commit will fail. After committing, you can view the commit history using the `log` command. Enter the command in Code Listing 3-53.

.. code-block:: bash

   $ git log

.. code-block:: text

   ---
   commit 243169b5b4bb18db534013340ca802b3954df55b (HEAD -> master)
   Author: Fish <fish@fishros.com>
   Date:   Tue Apr 18 00:52:27 2023 +0800

       Completed status publishing and display functionality

The commit history includes the commit ID, author name, email, commit date, and description. Congratulations, you have successfully completed your first code commit!