# SCRIPTS

Helpful scripts to create C++ and ROS projects

## Installation and usage

  * Download the scripts in a folder of your choice  
  `git clone https://github.com/angelsantamaria/scripts.git`  

  * Add the following line to your `bashrc` in order to execute the scripts from any console path  
  ``echo "source `pwd`/scripts/setup.bash" >> ~/.bashrc``

## Main scripts

#### C++ library project

Creates the main structure of a C++ library, including main .cpp and .h files, doxygen files and an example application in `src/examples`.

  * Example of usage  
  `create_cpp_project.sh -n Example` 

  * Options:
    - n: Project name
    - d: Project dependencies
    - h: Help

#### ROS package with a node 

Creates a ROS package with a node, written in C++, including separated files for an algorithm (.cpp and .h files) which can be ROS agnostic in a way to not depend on ROS versioning.

  * Example of usage  
  `create_ros_node.sh -n Example` 


