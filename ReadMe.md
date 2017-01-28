# SCRIPTS

Helpful scripts to create C++ and ROS projects

## C++ library project

Creates the main structure of a C++ library, including main .cpp and .h files, doxygen files and an example application in `src/examples`.

#### Installation and usage

  * Download the scripts in a folder of your choice  
  `git clone https://github.com/angelsantamaria/scripts.git`  

  * Add the following line to your `bashrc` in order to execute the scripts from any console path  
  `echo "source 'pwd'/scripts/setup.bash" >> ~/.bashrc`

  * Example of usage  
  `create_cpp_project.sh -n Example` 

  * Options:
    - n: Project name
    - d: Project dependencies
    - h: Help





