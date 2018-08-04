^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package visualstates
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2018-07-11)
------------------
* set the version of the ros package as 0.1.0
* Merge pull request `#55 <https://github.com/JdeRobot/VisualStates/issues/55>`_ from JdeRobot/ros-package
  Generated ROS package dependencies are fixed. fixes `#37 <https://github.com/JdeRobot/VisualStates/issues/37>`_
* cpp ros package can use python run time
* The generated code does not have runtime but depends on the visualstates package itself
* config dialog supports only ROS communication interface
* Merge pull request `#40 <https://github.com/JdeRobot/VisualStates/issues/40>`_ from JdeRobot/none-config-fix
  if config is none create it as jderobotconfig and fixes `#39 <https://github.com/JdeRobot/VisualStates/issues/39>`_
* if config is none create it as jderobotconfig
* Merge pull request `#36 <https://github.com/JdeRobot/VisualStates/issues/36>`_ from JdeRobot/ros-package
  VisualState tool is redesigned as a ROS package
* ros package of visualstates merging with the final code structure and improvements from master
* fixed remaining empty  merge characters in the source code
* merge from codegen-bug pull request of pushkal
* merge from pushkal's external ros package pull request
* restructure the project for ros
* format commands as source code
* change project directory to have everything under src directory
* Merge pull request `#21 <https://github.com/JdeRobot/VisualStates/issues/21>`_ from pushkalkatara/patch-1
  have quotation around path to handle path with with space
* Permissions to script in folders with space
  https://github.com/TheRoboticsClub/colab-gsoc2018-PushkalKatara/issues/8
  Solves `#8 <https://github.com/JdeRobot/VisualStates/issues/8>`_
* Merge pull request `#19 <https://github.com/JdeRobot/VisualStates/issues/19>`_ from JdeRobot/subautomata-remove-fix
  Fixing subautomata removal bug
* get the parent node from treemodel
* ignore precompiled python files pyc
* Merge pull request `#13 <https://github.com/JdeRobot/VisualStates/issues/13>`_ from JdeRobot/generated_code_dependencies
  Correcting installation path for standalone installation
* add installation directives to README file
* check ROS message directory
* fix the cmake installation path
* Merge pull request `#9 <https://github.com/JdeRobot/VisualStates/issues/9>`_ from JdeRobot/standalone_visualstates
  Fixing cmake paths
* fixing cmake paths
* Merge pull request `#8 <https://github.com/JdeRobot/VisualStates/issues/8>`_ from JdeRobot/standalone_visualstates
  Migration of VisualStates tool to its own repository
* added examples and updated installation cmake to copy examples under share folder
* ignore .idea directory
* apply small changes for having standalone visualstates
* Merge branch 'master' of github.com:JdeRobot/VisualStates
* first commit
* Solved minor bug in visualStates installation
* remove the sample folder inside visualStates, old examples will reside in src/examples/visualStates
* visualStates_py renamed to visualStates
* remove old visualstates
* `#993 <https://github.com/JdeRobot/VisualStates/issues/993>`_ refactored JderobotConfig.cmake included cmake wrappers
* Merge branch 'global/build/792-fix-warnings' of https://github.com/lr-morales/JdeRobot into lr-morales-global/build/792-fix-warnings
* class name change is fixed
* merge from master
* to be able to access cmakevar.hpp add build directory to include directories
* remove the generated file to prevent misunderstanding
* install glade file to general glade path, get cmake variables through a c++ class, fix broken library path after moving install prefix to /opt/jderobot
* merge from jderobot_comm
* [Issue `#792 <https://github.com/JdeRobot/VisualStates/issues/792>`_] Removes compilation warnings from tool visualStates
* [issue `#890 <https://github.com/JdeRobot/VisualStates/issues/890>`_] Solved installation bug in visualStates
* [issue `#879 <https://github.com/JdeRobot/VisualStates/issues/879>`_] solved bug in a visualStates file install
* [issue `#879 <https://github.com/JdeRobot/VisualStates/issues/879>`_] Solved bug in some libraries installation
* [issue `#879 <https://github.com/JdeRobot/VisualStates/issues/879>`_] Updated visualStates.cpp bug with getinterfaces.sh location
* [issue `#879 <https://github.com/JdeRobot/VisualStates/issues/879>`_] updated visualStates.cpp for installation in /opt/jderobot
* [issue `#879 <https://github.com/JdeRobot/VisualStates/issues/879>`_] moved all installation of libs to /opt/jderobot/libs instead of /opt/jderobot/libs/jderobot/
* [issue `#879 <https://github.com/JdeRobot/VisualStates/issues/879>`_] moved all installations to /opt/jderobot
* change name visualHFSM in variable and class names to visualStates
* visualHFSM name is changed to visualStates name in the source code
* Contributors: AAAAAAAAA, Aitor Martinez, Aitor Martínez Fernández, Francisco Pérez, Francisco Rivas, Jose Maria Canas Plaza, Luis Roberto Morales Iglesias, Okan Aşık, Pushkal Katara
