#!/bin/bash

# check wether the scripts path environment variable has been defined
scripts_path=`echo "${SCRIPTS_PATH}"`
if [[ -z "${scripts_path}" ]]
then
  echo "The scripts path environment variable has not been defined. Please see the wiki documentation for instructions on how to create it."
  exit
else
  echo "The scripts path environment variable has been properly defined."
fi

source "${SCRIPTS_PATH}/libraries/scripts_library.sh"

function check_server_file_integrity
{
  local node_h=$1
  local node_c=$2
  local comment=""

  # check the node.h file
  comment="\[service client headers\]"
  find_comment_in_file "${comment}" "${node_h}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[service client headers\] from the header file ${node_h}"
  fi
  comment="\[service attributes\]"
  find_comment_in_file "${comment}" "${node_h}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[service attributes\] from the header file ${node_h}"
  fi

  # check the node.cpp file
  comment="\[init services\]"
  find_comment_in_file "${comment}" "${node_c}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[init services\] from the header file ${node_c}"
  fi 
  comment="\[service callbacks\]"
  find_comment_in_file "${comment}" "${node_c}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[service callbacks\] from the header file ${node_c}"
  fi 
  comment="\[free dynamic memory\]"
  find_comment_in_file "${comment}" "${node_c}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[free dynamic memory\] from the header file ${node_c}"
  fi 
}

function check_server_attributes_functions
{
  local node_h=$1
  local node_c=$2
  local server_name=$3
  local srv_pkg=$4
  local srv_file=$5
  local topic_name=$6
  local callback=$7
  local mutex_name=$8
  local class_name=$9
  local line=""

  # check the node.h file
  line="ros::ServiceServer ${server_name};"
  find_comment_in_file "${line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A service server with the same name is already declared in file ${node_h} line ${line_number}"
  fi
  line="bool ${callback}(${srv_pkg}::${srv_file}::Request &req, ${srv_pkg}::${srv_file}::Response &res);"
  find_comment_in_file "${line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A service callback function with the same name is already declared in file ${node_h} line ${line_number}"
  fi
  line="pthread_mutex_t ${mutex_name};"
  find_comment_in_file "${line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A mutex with the same name is already declared in file ${node_h} line ${line_number}"
  fi
  line="void ${mutex_name}enter(void);"
  find_comment_in_file "${line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A mutex enter function with the same name is already declared in file ${node_h} line ${line_number}"
  fi
  line="void ${mutex_name}exit(void);"
  find_comment_in_file "${line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A mutex exit function with the same name is already declared in file ${node_h} line ${line_number}"
  fi

  # check the node.cpp file
  line="this->${server_name} = nh.advertiseService(\"${topic_name}\", &${class_name}::${callback}, this);"
  find_comment_in_file "${line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A service server with the same name is already initialized in file ${node_c} line ${line_number}"
  fi
  line="pthread_mutex_init(&this->${mutex_name},NULL);"
  find_comment_in_file "${line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A mutex with the same name is already initialized in file ${node_c} line ${line_number}"
  fi
  line="pthread_mutex_destroy(&this->${mutex_name});"
  find_comment_in_file "${line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A mutex with the same name is already destroyed in file ${node_c} line ${line_number}"
  fi
  aux_line="bool ${class_name}::${callback}(${srv_pkg}::${srv_file}::Request &req, ${srv_pkg}::${srv_file}::Response &res)"
  find_comment_in_file "${aux_line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A service callback function with the same name is already implemented in file ${node_c} line ${line_number}"
  fi
  line="void ${class_name}::${mutex_name}enter(void)"
  find_comment_in_file "${line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A mutex enter function  with the same name is already implemented in file ${node_c} line ${line_number}"
  fi
  line="void ${class_name}::${mutex_name}exit(void)"
  find_comment_in_file "${line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A mutex exit function  with the same name is already implemented in file ${node_c} line ${line_number}"
  fi
}

function create_service_server
{
  #read input params
  local ros_pkg=$1
  local topic_name=$2
  local srv_file=$3
  local srv_pkg=$4
  local node_h=$5
  local node_c=$6
  local driver_alg=$7
  local server_name="${topic_name}_server_"
  local mutex_name="${topic_name}_mutex_"
  local callback="${topic_name}Callback"
  local line=""
  local aux_line=""
  local comment=""

  #get class basename
  get_class_basename "${node_c}"
  if [[ -z ${class_name} ]]
  then
    kill_exit "impossible to retrieve class basename"
  fi
# echo "class_name=${class_name}"

  #check files integrity before making any changes
  check_cmakelists_file_integrity "${driver_alg}"
  check_server_file_integrity "${node_h}" "${node_c}"
  check_server_attributes_functions "${node_h}" "${node_c}" "${server_name}" "${srv_pkg}" "${srv_file}" "${topic_name}" "${callback}" "${mutex_name}" "${class_name}"

################################################################################
#modify Node.h

  #look for include files and add them if are not already there
  line="#include <${srv_pkg}/${srv_file}.h>"
  comment="\[service client headers\]"
  add_line_to_file "${line}" "${comment}" "${node_h}"

  aux_line="ros::ServiceServer ${server_name};"
  line="\ \ \ \ ${aux_line}\n"
  aux_line="bool ${callback}(${srv_pkg}::${srv_file}::Request &req, ${srv_pkg}::${srv_file}::Response &res);"
  line="${line}\ \ \ \ ${aux_line}\n"
  aux_line="pthread_mutex_t ${mutex_name};"
  line="${line}\ \ \ \ ${aux_line}\n"
  aux_line="void ${mutex_name}enter(void);"
  line="${line}\ \ \ \ ${aux_line}\n"
  aux_line="void ${mutex_name}exit(void);"
  line="${line}\ \ \ \ ${aux_line}\n"
  comment="\[service attributes\]"
  add_line_to_file "${line}" "${comment}" "${node_h}"

################################################################################
#modify Node.cpp

  aux_line="this->${server_name} = nh.advertiseService(\"${topic_name}\", &${class_name}::${callback}, this);"
  line="\ \ ${aux_line}\n"
  aux_line="pthread_mutex_init(&this->${mutex_name},NULL);\n"
  line="${line}\ \ ${aux_line}"
  comment="\[init services\]"
  add_line_to_file "${line}" "${comment}" "${node_c}"

  aux_line="bool ${class_name}::${callback}(${srv_pkg}::${srv_file}::Request &req, ${srv_pkg}::${srv_file}::Response &res)"
  line="${aux_line}\n"
  aux_line="{"
  line="${line}${aux_line}\n"
  aux_line="\ \ ROS_INFO(\"${class_name}::${callback}: New Request Received!\");"
  line="${line}${aux_line}\n\n"
  aux_line="\ \ //use appropiate mutex to shared variables if necessary"
  line="${line}${aux_line}\n"
  aux_line="\ \ //this->${driver_alg}_.lock();"
  line="${line}${aux_line}\n"
  aux_line="\ \ //this->${mutex_name}enter();"
  line="${line}${aux_line}\n\n"
  aux_line="\ \ //ROS_INFO(\"${class_name}::${callback}: Processing New Request!\");"
  line="${line}${aux_line}\n"
  aux_line="\ \ //do operations with req and output on res"
  line="${line}${aux_line}\n"
  aux_line="\ \ //res.data2 = req.data1 + my_var;"
  line="${line}${aux_line}\n\n"
  aux_line="\ \ //unlock previously blocked shared variables"
  line="${line}${aux_line}\n"
  aux_line="\ \ //this->${mutex_name}exit();"
  line="${line}${aux_line}\n"
  aux_line="\ \ //this->${driver_alg}_.unlock();"
  line="${line}${aux_line}\n\n"
  aux_line="\ \ return true;"
  line="${line}${aux_line}\n"
  aux_line="}"
  line="${line}${aux_line}\n\n"
  aux_line="${template_class}void ${class_name}::${mutex_name}enter(void)"
  line="${line}${aux_line}\n"
  aux_line="{"
  line="${line}${aux_line}\n"
  aux_line="\ \ pthread_mutex_lock(&this->${mutex_name});"
  line="${line}${aux_line}\n"
  aux_line="}"
  line="${line}${aux_line}\n\n"
  aux_line="${template_class}void ${class_name}::${mutex_name}exit(void)"
  line="${line}${aux_line}\n"
  aux_line="{"
  line="${line}${aux_line}\n"
  aux_line="\ \ pthread_mutex_unlock(&this->${mutex_name});"
  line="${line}${aux_line}\n"
  aux_line="}"
  line="${line}${aux_line}\n"
  comment="\[service callbacks\]"
  add_line_to_file "${line}" "${comment}" "${node_c}"

  line="pthread_mutex_destroy(&this->${mutex_name});"
  comment="\[free dynamic memory\]"
  add_line_to_file "\ \ ${line}" "${comment}" "${node_c}"

################################################################################
# Modify package.xml
# check if the message package is the current ros package

  add_build_run_dependencies "${ros_pkg}" "${srv_pkg}"

################################################################################
# modify the CMakeLists.txt file
# check if the message package is the current ros package

  add_cmake_dependencies "${ros_pkg}" "${srv_pkg}"

################################################################################
#compile
  goto_catkin_workspace
  #catkin_make --only-pkg-with-deps ${ros_pkg}
  catkin build ${ros_pkg}
}
