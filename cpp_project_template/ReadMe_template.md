# project_name 


## Introduction 

#### Pre-Requisites

This package requires of the following libraries and packages  

  * [cmake](http://www.cmake.org), a cross-platform build system
  * [doxygen](http://www.doxygen.org) and 
    [graphviz](http://www.graphviz.org) to generate the documentation
  * stdc++  

Under linux all of these utilities are available in ready-to-use packages.  

Under MacOS most of the packages are available via [fink](http://www.finkproject.org).  

#### Compilation

  Just download this package, uncompress it, and execute  

  `cd build`  
  `cmake ..`  

to generate the makefile and then  

  `make`  

to obtain the shared library and also all the example programs.  

The *cmake* only needs to be executed once (make will automatically call *cmake* 
if you modify one of the `CMakeList.txt` files).  

To generate this documentation type  

  `make doc`  

The files in the `build` directory are genetated by *cmake* and *make* 
and can be safely removed.  
After doing so you will need to call cmake manually again.  

#### Configuration  

The default build mode is DEBUG. That is, objects and executables include debug information.  

The RELEASE build mode optimizes for speed. To build in this mode execute  

  `cmake .. -DCMAKE_BUILD_TYPE=RELEASE`  

The release mode will be kept until next time cmake is executed.  

#### Installation

In order to be able to use the library, it it necessary to copy it into the system. 
To do that, execute  

  `make install`  

as root and the shared libraries will be copied to `/usr/local/lib/<project_name>` directory
and the header files will be copied to `/usr/local/include/<project_name>` directory. At 
this point, the library may be used by any user.  

To remove the library from the system, exceute  

  `make uninstall`  

as root, and all the associated files will be removed from the system.  

## Customization  

To build a new application using these library, first it is necessary to locate if the library
has been installed or not using the following command in your `CMakeLists.txt`  

  `FIND_PACKAGE(library_name REQUIRED)`  

In the case that the package is present, it is necessary to add the header files directory to
the include directory path by using  

  `INCLUDE_DIRECTORIES(${library_name_INCLUDE_DIR})`

Finally, it is also nevessary to link with the desired libraries by using the following command  

  `TARGET_LINK_LIBRARIES(<executable name> ${library_name_LIBRARY})`

## License

This package is licensed under a [GPL 3.0 License](http://www.gnu.org/licenses/gpl.html)

#### Disclaimer

Copyright (C) 2017
http://www.angelsantamaria.eu
All rights reserved.

This file is part of project_name library  

project_name library is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>


