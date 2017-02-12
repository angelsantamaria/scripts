#!/bin/bash

# WET

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
source "${SCRIPTS_PATH}/libraries/create_action_server.sh"
source "${SCRIPTS_PATH}/libraries/create_action_client.sh"

check_libraries
check_templates

echo ""
echo "/*********************************************/"
echo "/*          Creating New ROS Action          */"
echo "/*********************************************/"

server_client=
ros_pkg=
action_name=
action_file=

#check for input project name paramenter
while getopts “:o:p:a:m:” OPTION
do
  case $OPTION in
    o)
       server_client=$OPTARG
       ;;
    p)
       ros_pkg=$OPTARG
       ;;
    a)
       action_name=$OPTARG
       ;;
    m)
       action_file=$OPTARG
       ;;
    ?)
       echo "invalid input argument ${OPTION}"
       kill_exit "Usage: add_action_server_client.sh -o [server,client] -p ros_pkg -a action_name -m message.action"
       exit
       ;;
  esac
done

#check if publisher name parameter is filled up
if [ ! "${server_client}" ] || [ ! "${ros_pkg}" ] || [ ! "${action_name}" ] || [ ! "${action_file}" ]
then
  echo "Missing input parameters..."
  kill_exit "Usage: add_action_server_client.sh -o [server,client] -p ros_pkg -a action_name -m message.action"
fi

#check server client parameter
if [[ ! "${server_client}" = "server" ]] && [[ ! "${server_client}" = "client" ]]
then
  kill_exit "First parameter must be either \"server\" or \"client\", aborting ..."
fi

#check if package exists
check_package "${ros_pkg}"
if [[ ${pkg_exists} == true ]]
then
  roscd ${ros_pkg}
else
  kill_exit "ROS package ${ros_pkg} does NOT exist yet, please first run create_ros_node.sh"
fi

#validate file extension .action
act="action"
ext=${action_file##*.}
ext=$(echo ${ext} | tr "[:upper:]" "[:lower:]")
action_file=${action_file%.*}

#check extension
if [[ ! "${ext}" = "${act}" ]]
then
  kill_exit "Wrong file extension, please provide a .action file, aborting ..."
fi

#look for ACTION file
find_ros_message ${action_file} ${act} ${ros_pkg}
if ${found_it}
then
  action_file=${my_file}
  echo "ACTION file ${action_file} found!"
else
  echo "ACTION file ${action_file} does NOT exist, please check if file is in valid directories"
  kill_exit "Aborting ..."
fi

#retrieve header and source files
get_h_cpp_files ${ros_pkg}
echo "node_h=${node_h}"
echo "node_c=${node_c}"
driver_alg="alg"

if [[ -z ${node_h} ]] || [[ -z ${node_c} ]] || [[ -z ${driver_alg} ]]
then
  kill_exit "Problems with headers and/or source files"
fi

#go to package folder
roscd "${ros_pkg}"

#modify node files adding server/client parameters
if [[ "${server_client}" = "server" ]]
then
  create_action_server ${ros_pkg} ${action_name} ${action_file%.action} ${file_pkg} ${node_h} ${node_c} ${driver_alg}
else
  create_action_client ${ros_pkg} ${action_name} ${action_file%.action} ${file_pkg} ${node_h} ${node_c} ${driver_alg}
fi

