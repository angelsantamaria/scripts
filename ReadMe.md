# SCRIPTS

Helpful scripts to create C++ and ROS projects

## Installation and usage

  * Download the scripts in a folder of your choice  
  `git clone https://github.com/angelsantamaria/scripts.git`  

  * Run the following line to set your `bashrc`. This will allow you to execute the scripts from any console path.  
  ``echo "source `pwd`/scripts/setup.bash" >> ~/.bashrc``

## Main scripts

#### C++ library project

Creates the main structure of a C++ library, including main .cpp and .h files, doxygen files and an example application in `src/examples`. 

  * Example of usage  
  `create_cpp_project.sh -n example` 

  * Options:
    - n: Project name
    - d: Project dependencies
    - h: Help

#### ROS package with a node 

Creates a ROS package with a node inside the current path, written in C++, including separated files for an algorithm (.cpp and .h files) which can be ROS agnostic in a way to not depend on ROS versioning.

  * Example of usage  
  `create_ros_node.sh -n example` 

#### ROS topic publisher/subscriber

Adds a publisher/subscriber to an existing node, written in C++.

  * Example of usage  
  `add_publisher_subscriber.sh -o publisher -p example_pkg -t example_pub -m std_msgs/String.msg -b 1` 

  * Options:
    - o: Type (publisher/subscriber)
    - p: Node inwhich the publisher/subscriber will be added
    - t: Publisher/subscriber name inside the code
    - m: Message type
    - b: Buffer length

#### ROS service client/server

Adds a service client/server to an existing node, written in C++.

  * Example of usage  
  `add_service_server_client.sh -o client -p example -s example_srv -m Empty.srv`

  * Options:
    - o: Type (client/server)
    - p: Node inwhich the service client/server will be added
    - s: Service client/server name inside the code
    - m: Service message type

#### ROS action client/server

Adds a action client/server to an existing node, written in C++.

  * Example of usage  
  `add_action_server_client.sh -o client -p example -s example_action -m Test.action`

  * Options:
    - o: Type (client/server)
    - p: Node inwhich the service client/server will be added
    - a: Action client/server name inside the code
    - m: Action message type
