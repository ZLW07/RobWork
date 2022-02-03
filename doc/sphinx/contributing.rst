.. _contributing:

************
Contributing
************

When contributing to the RobWork project as a developer, you need to follow the guidelines provided here.
You should `Setup your IDE`_ to follow the :ref:`coding_standard`. Please read the coding guidelines carefully.
Finally, some guidelines is given on `Committing Code`_ to our main repository.


Setup your IDE
==============

If you use one of the following IDEs, please consider importing one of the provided style configurations:

- :download:`Eclipse IDE <../rw_codestyle_eclipse.xml>`
- :download:`QtCreator IDE <../rw_codestyle_qtcreator.xml>`
- :download:`CLion IDE <../rw_codestyle_clion.xml>`
- :download:`Visual Studio IDE <../rw_codestyle_visualstudio.vssettings>`

In case you are developing as a part of SDU Robotics, please configure your editor to add the RobWork license to your .cpp and .hpp files::

  /******************************************************************************
   * Copyright 2019 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
   * Faculty of Engineering, University of Southern Denmark
   *
   * Licensed under the Apache License, Version 2.0 (the "License");
   * you may not use this file except in compliance with the License.
   * You may obtain a copy of the License at
   *
   *     http://www.apache.org/licenses/LICENSE-2.0
   *
   * Unless required by applicable law or agreed to in writing, software
   * distributed under the License is distributed on an "AS IS" BASIS,
   * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   * See the License for the specific language governing permissions and
   * limitations under the License.
   ******************************************************************************/

If you develop under a different license, your code might not be suitable for addition to the main RobWork repository before additional review.

.. toctree::
   :maxdepth: 2
   :caption: Coding Standards:

   contributing/coding_standard
   contributing/cmake_standard

Committing Code
===============

In order to contribute with code and fixes to the main RobWork project, you have to first make a personal fork of the project.
In this fork you can do your own work.
After pushing your work to you personal project, it is then possible to make a GitLab merge request in order to merge the changes into the main project.
A merge request will require some checks to succeed, and will need approval by maintainers at SDU Robotics, before being merged.

The full process is as follows:

#. Fork the repository.
   Go to https://gitlab.com/sdurobotics/RobWork and click Fork in the upper right corner.
   Follow the guide and place it somewhere in your own namespace.
#. Clone your new forked project to your local machine, using either SSH or HTTP:

   - **git clone git@gitlab.com:<username>/RobWork.git**
   - **git clone https://gitlab.com/<username>/RobWork.git**

#. Go to your cloned project folder and add the upstream project as an extra remote, using either SSH or HTTPS:

   - **git remote add upstream git@gitlab.com:sdurobotics/RobWork.git**
   - **git remote add upstream https://gitlab.com/sdurobotics/RobWork.git**

#. To update your project, checkout your own master, fetch changes from upstream, and merge them into your own master.
   Optionally, you can push your new updated master to your own fork on GitLab.com:

   - **git checkout master**
   - **git fetch upstream**
   - **git merge upstream/master**
   - **git push origin master**

#. Before adding your work, checkout a new branch with a meaningful name for your changes:

   - **git checkout -b <branch_name>**

#. Use **git status** to check if there is uncommitted work on your branch. Use **git add** to stage changes for commit. You can check what is staged with **git status** and **git diff --cached**.
#. Test that you can still compile the RobWork projects locally. Run the tests to make sure that you did not break any tests.
#. If everything seems to work, commit your staged changes with **git commit -m "Some meaningful description of your changes"**.
   If you have not set your username and email, you will be guided on how to do so.
#. After you have made your commits, push the commit to your project at GitLab with **git push origin <branch_name>**.
#. Go to your project page on GitLab.com and click on "Merge Requests" and "New merge request".
#. Select the source branch from your own project and the target branch sdurobotics/RobWork master branch.
#. Click "Compare branches and continue". Put in a description and finalize your merge request.
#. When the merge request is submitted, a pipeline is launched. This pipeline tests that the most important parts of RobWork compile (a minimal compilation).
#. If the pipeline fails, fix the error, commit the change, and push it to your branch. The new change will cause the pipeline to run again.
#. When the pipeline succeeds, you will have to wait for a maintainer from SDU Robotics to accept the request.
   When the request is accepted and the pipeline succeeds, you will be able to finish the merge (this step might also be done directly by a maintainer).
#. After the merge, a much larger pipeline is triggered. This checks that RobWork compiles and that tests still work. This is done on all platforms supported by RobWork.
#. If this last pipeline fails, you should fix the issue as quickly as possible. You will need to create a new merge request with a fix.

When you commit:

- Always specify a commit message stating what was changed (this makes it possible for other people to see why you made changes and, more importantly, why you changed it)
