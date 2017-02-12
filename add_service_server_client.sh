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
source "${SCRIPTS_PATH}/libraries/create_service_server.sh"
source "${SCRIPTS_PATH}/libraries/create_service_client.sh"

check_libraries
check_templates

echo ""
echo "/*********************************************/"
echo "/*      Creating New ROS Server/Client       */"
echo "/*********************************************/"

server_client=
ros_pkg=
service_name=
srv_file=

#check for input project name paramenter
while getopts “:o:p:s:m:” OPTION
do
  case $OPTION in
    o)
       server_client=$OPTARG
       ;;
    p)
       ros_pkg=$OPTARG
       ;;
    s)
       service_name=$OPTARG
       ;;
    m)
       srv_file=$OPTARG
       ;;
    ?)
       echo "invalid input argument ${OPTION}"
       kill_exit "Usage: add_server_client.sh -o [server,client] -p ros_pkg -s service_name -m service.srv"
       exit
       ;;
  esac
done

#check if publisher name parameter is filled up
if [ ! "${server_client}" ] || [ ! "${ros_pkg}" ] || [ ! "${service_name}" ] || [ ! "${srv_file}" ]
then
  echo "Missing input parameters..."
  kill_exit "Usage: add_server_client.sh -o [server,client] -p ros_pkg -s service_name -m service.srv"
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

#validate file extension .srv
srv="srv"
ext=${srv_file##*.}
ext=$(echo ${ext} | tr "[:upper:]" "[:lower:]")

#check extension
if [[ ! "${ext}" = "${srv}" ]]
then
  kill_exit "Wrong file extension, please provide a .srv file, aborting ..."
fi

#look for SRV file
find_ros_message ${srv_file} ${srv} ${ros_pkg}
if ${found_it}
then
  srv_file=${my_file}
  echo "SRV file ${srv_file} found!"
else
  echo "SRV file ${srv_file} does NOT exist, please check if file is in valid directories"
  kill_exit "Aborting ..."
fi

#retrieve header and source files
get_h_cpp_files ${ros_pkg}
echo "node_h=${node_h}"
echo "node_c=${node_c}"

if [[ -z ${node_h} ]] || [[ -z ${node_c} ]]
then
  kill_exit "Problems with headers and/or source files"
fi

#go to package folder
roscd "${ros_pkg}"

#modify node files adding server/client parameters
if [[ "${server_client}" = "server" ]]
then
  create_service_server ${ros_pkg} ${service_name} ${srv_file%.srv} ${file_pkg} ${node_h} ${node_c}
else
  create_service_client ${ros_pkg} ${service_name} ${srv_file%.srv} ${file_pkg} ${node_h} ${node_c}
fi
