#!/bin/bash

# WET

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
source "${SCRIPTS_PATH}/libraries/create_publisher.sh"
source "${SCRIPTS_PATH}/libraries/create_subscriber.sh"

check_libraries
check_templates

echo ""
echo "/*********************************************/"
echo "/*   Creating New ROS Publisher/Subscriber   */"
echo "/*********************************************/"

pub_subs=
ros_pkg=
topic_name=
msg_file=
buffer=

#check for input project name paramenter
while getopts “:o:p:t:m:b:” OPTION
do
  case $OPTION in
    o)
       pub_subs=$OPTARG
       ;;
    p)
       ros_pkg=$OPTARG
       ;;
    t)
       topic_name=$OPTARG
       ;;
    m)
       msg_file=$OPTARG
       ;;
    b)
       buffer=$OPTARG
       ;;
    ?)
       echo "invalid input argument ${OPTION}"
       kill_exit "Usage: add_publisher_subscriber.sh -o [publisher,subscriber] -p ros_pkg -t topic_name -m message.msg -b 100"
       exit
       ;;
  esac
done

#check if publisher name parameter is filled up
if [ ! "${pub_subs}" ] || [ ! "${ros_pkg}" ] || [ ! "${topic_name}" ] || [ ! "${msg_file}" ] || [ ! "${buffer}" ]
then
  echo "Missing input parameters..."
  kill_exit "Usage: add_publisher_subscriber.sh -o [publisher,subscriber] -p ros_pkg -t topic_name -m message.msg -b 100"
fi

#check publisher subscriber parameter
if [[ ! "${pub_subs}" = "publisher" ]] && [[ ! "${pub_subs}" = "subscriber" ]]
then
  kill_exit "First parameter must be either \"publisher\" or \"subscriber\", aborting ..."
fi

#check if package exists
result=`roscd ${ros_pkg}`
if [[ -z "${result}" ]]
then
  roscd ${ros_pkg}
else
  kill_exit "ROS package ${ros_pkg} does NOT exist yet, please first run create_ros_node.sh"
fi

check_package "${ros_pkg}"
if [[ ${pkg_exists} == true ]]
then
  roscd ${ros_pkg}
else
  kill_exit "ROS package ${ros_pkg} does NOT exist yet, please first run create_ros_node.sh"
fi

#validate file extension .msg
msg="msg"
ext=${msg_file##*.}
ext=$(echo ${ext} | tr "[:upper:]" "[:lower:]")

#check extension
if [[ ! "${ext}" = "${msg}" ]]
then
  kill_exit "Wrong file extension, please provide a .msg file, aborting ..."
fi

#look for MSG file
find_ros_message ${msg_file} ${msg} ${ros_pkg}
if ${found_it}
then
  msg_file=${my_file}
  echo "MSG file ${msg_file} found!"
else
  echo "MSG file ${msg_file} does NOT exist, please check if file is in valid directories"
  kill_exit "Aborting ..."
fi

# Sanitize input and assign to new variable
export clean_buffer=`echo "${buffer}" | tr -cd '[:digit:]'`

#check if buffer parameter is filled up
if [ "$clean_buffer" ] && [ "$clean_buffer -gt 0" ]
then
  echo "Setting buffer length to $clean_buffer"
else
  kill_exit "No buffer provided, aborting ..."  
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
if [[ "${pub_subs}" = "publisher" ]]
then
  create_publisher ${ros_pkg} ${topic_name} ${msg_file%.msg} ${file_pkg} ${buffer} ${node_h} ${node_c} ${driver_alg}
else
  create_subscriber ${ros_pkg} ${topic_name} ${msg_file%.msg} ${file_pkg} ${buffer} ${node_h} ${node_c} ${driver_alg}
fi

