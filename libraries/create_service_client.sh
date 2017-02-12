#!/bin/bash

# check wether the scripts path environment variable has been defined
scripts_path=`echo "${SCRIPTS_PATH}"`
if [[ -z "${scripts_path}" ]]
then
  echo "The scripts path environment varibale has not been defined. Please see the wiki documentation for instructions on how to create it."
  exit
else
  echo "The scripts path environment variable has been properly defined."
fi

source "${SCRIPTS_PATH}/libraries/scripts_library.sh"

function check_client_file_integrity
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
  comment="\[client attributes\]"
  find_comment_in_file "${comment}" "${node_h}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[client attributes\] from the header file ${node_h}"
  fi
  # chekc the node.cpp file
  comment="\[init clients\]"
  find_comment_in_file "${comment}" "${node_c}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[init clients\] from the header file ${node_c}"
  fi
  comment="\[fill srv structure and make request to the server\]"
  find_comment_in_file "${comment}" "${node_c}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[fill srv structure and make request to the server\] from the header file ${node_c}"
  fi
}

function check_client_attributes_functions
{
  local node_h=$1
  local node_c=$2
  local client_name=$3
  local srv_pkg=$4
  local srv_file=$5
  local topic_name=$6
  local line=""

  # check the node.h file
  line="ros::ServiceClient ${client_name};"
  find_comment_in_file "${line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A service client with the same name is already declared in file ${node_h} line ${line_number}"
  fi
  line="${srv_pkg}::${srv_file} ${topic_name}_srv_;"
  find_comment_in_file "${line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A service message with the same name is already declared in file ${node_h} line ${line_number}"
  fi

  # check the node.cpp file
  line="${client_name} = nh.serviceClient<${srv_pkg}::${srv_file}>(\"${topic_name}\");"
  echo "${line}"
  find_comment_in_file "${line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A service client with the same name is already initialized in file ${node_c} line ${line_number}"
  fi
  line="if (${client_name}.call(${topic_name}_srv_))"
  find_comment_in_file "${line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A service with the same name is already called in file ${node_c} line ${line_number}"
  fi
}

function create_service_client
{
  #read input params
  local ros_pkg=$1
  local topic_name=$2
  local srv_file=$3
  local srv_pkg=$4
  local node_h=$5
  local node_c=$6
  local driver_alg=$7
  local client_name="${topic_name}_client_"
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
  check_client_file_integrity "${node_h}" "${node_c}"
  check_client_attributes_functions "${node_h}" "${node_c}" "${client_name}" "${srv_pkg}" "${srv_file}" "${topic_name}" 

################################################################################
#modify Node.h

  #look for include files and add them if are not already there
  line="#include <${srv_pkg}/${srv_file}.h>"
  comment="\[service client headers\]"
  add_line_to_file "${line}" "${comment}" "${node_h}"

  aux_line="ros::ServiceClient ${client_name};"
  line="\ \ \ \ ${aux_line}\n"
  aux_line="${srv_pkg}::${srv_file} ${topic_name}_srv_;"
  line="${line}\ \ \ \ ${aux_line}\n"

  comment="\[client attributes\]"
  add_line_to_file "${line}" "${comment}" "${node_h}"

################################################################################
#modify Node.cpp

  line="${client_name} = nh.serviceClient<${srv_pkg}::${srv_file}>(\"${topic_name}\");"
  comment="\[init clients\]"
  add_line_to_file "\ \ ${line}\n" "${comment}" "${node_c}"

  aux_line="//${topic_name}_srv_.request.data = my_var;"
  line="\ \ ${aux_line}\n"
  aux_line="\ \ //ROS_INFO(\"${class_name}:: Sending New Request!\");"
  line="${line}${aux_line}\n"
  aux_line="\ \ //if (${client_name}.call(${topic_name}_srv_))"
  line="${line}${aux_line}\n"
  aux_line="\ \ //{"
  line="${line}${aux_line}\n"
  aux_line="\ \ \ \ //ROS_INFO(\"${class_name}:: Response: %s\", ${topic_name}_srv_.response.result);"
  line="${line}${aux_line}\n"
  aux_line="\ \ //}"
  line="${line}${aux_line}\n"
  aux_line="\ \ //else"
  line="${line}${aux_line}\n"
  aux_line="\ \ //{"
  line="${line}${aux_line}\n"
  aux_line="\ \ \ \ //ROS_INFO(\"${class_name}:: Failed to Call Server on topic ${topic_name} \");"
  line="${line}${aux_line}\n"
  aux_line="\ \ //}"
  line="${line}${aux_line}\n"
  comment="\[fill srv structure and make request to the server\]"
  add_line_to_file "${line}\n" "${comment}" "${node_c}"

################################################################################

echo ""
echo "${ros_pkg}"
echo ""
echo "${srv_pkg}"

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
  catkin_make --only-pkg-with-deps ${ros_pkg}
}

