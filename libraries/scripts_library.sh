#!/bin/bash

source "${ROS_ROOT}/../rosbash/rosbash" || kill_exit "ROS_ROOT Not found, try to install ROS again"

# kill_exit
# prints string in $1 and exits script
# - $1: string to print
function kill_exit
{
  echo -e $1
  echo ""
  exit
}

# check_libraries
# looks for other scripts in the library. If one script has 
# not been found exits the script
function check_libraries
{
  echo "checking libraries ..."
  if [[ -d "${SCRIPTS_PATH}/libraries" ]]
  then
    pushd "${SCRIPTS_PATH}/libraries"
    if [[ -e "create_service_server.sh" ]] && [[ -e "create_service_client.sh" ]] && [[ -e "create_publisher.sh" ]] && [[ -e "create_subscriber.sh" ]]
    then
      echo "All library files available"
      popd
    else
      popd
      kill_exit "Missing some script files, please download the scripts again, aborting ..."
    fi
  else
    popd
    kill_exit "Missing libraries folder, please download the scripts again, aborting ..."
  fi
}

# check_templates
# looks for template files in the library. If one template is
# missing then exits the script
function check_templates
{
  echo "checking algorithm templates ..."
  if [[ -d "${SCRIPTS_PATH}/ros_node_templates" ]]
  then
    pushd "${SCRIPTS_PATH}/ros_node_templates"
    if [[ -e "CMakeLists.txt" ]] && [[ -e "template_alg.cfg" ]] && [[ -e "template_alg.cpp" ]] && [[ -e "template_alg.h" ]] && [[ -e "template_node.cpp" ]] && [[ -e "template_node.h" ]]
    then
      echo "Generic algorithm template files available"
      popd
    else
      popd
      kill_exit "Missing some generic algorithm template files, please download the scripts again, aborting ..."
    fi
  else
    popd
    kill_exit "Missing generic algorithm templates folder, please download the scripts again, aborting ..."
  fi
}

# check_package
# looks for package in $1. Tries to access package via roscd.
# If an error is returned, pkg does NOT exists
function check_package
{
  if [[ -z $1 ]]
  then
    kill_exit "check_package: missing input parameters"
  fi

  #save "roscd pkg" output
  local out=`roscd $1`

  #if no output ==> pkg exists
  if [[ -z "${out}" ]]
  then
    pkg_exists=true
  else
    pkg_exists=false
  fi
}

# create_basename
# Given a package name without spaces, creates the corresponding basename
# - $1: package name
function create_basename
{
  if [[ -z $1 ]]
  then
    kill_exit "create_basename: missing input parameters"
  fi

  local in_pkg=$1

  #parse input name for '_'
  local parsed_name=$(echo ${in_pkg} | sed 's/_/ /g')

  #for each word, uppercase the first letter and add it to basename
  for ii in ${parsed_name}
  do
    local first=$(echo ${ii:0:1} | tr "[:lower:]" "[:upper:]")
    basename="${basename}${first}${ii:1}"
  done
  #echo "basename=${basename}"
}

# get_core_basename
# Given a source file, retrieves the class name
# Looks for class destructor
# if no destrctor defined, function fails
function get_class_basename
{
  if [[ -z $1 ]]
  then
    kill_exit "create_basename: missing input parameters"
  fi

  local source_file=$1
  local class_name_def=`sed -n "/::~ */p" ${source_file}`
  local last_char_pos=`expr index "${class_name_def}" :`
  let last_char_pos-=1
  class_name=`expr substr "${class_name_def}" 1 ${last_char_pos}`
}

# browse_files
# Looks for all files in a folder to match the file in $1 with extension $2.
# - $1: file to find
# - $2: file extension
function browse_files
{
  if [[ -z $1 ]] || [[ -z $2 ]]
  then
    kill_exit "check_package: missing input parameters"
  fi

  local my_file=$1
  local ext=$2
  local ls_file=

  # for all files in current directory
  for ls_file in `ls`
  do
  # check if file extension is the same
  if [[ ${ls_file##*.} =  ${ext} ]]
  then
    # check if filename is the same
    if [[ ${my_file} = ${ls_file} ]]
    then
    file_path=`pwd`
    found_it=true
    break
    fi
  fi
  done
}

# find_ros_message
# Validates existance of the given ros message type file in $1 with 
# extension $2. When $2=action then looks for the 7 .msg created files.
# Adds message package to project package.xml if required.
# - $1: name of the ros message file to find
# - $2: type of the ros message file: msg, srv or action
# - $3: name of the ros pkg who wants to search the file
#       (for searching package.xml to add dependency)
function find_ros_message
{
  if [[ -z $1 ]] || [[ -z $2 ]] || [[ -z $3 ]]
  then
    kill_exit "find_ros_message: missing input parameters"
  fi

  # read input parameters
  my_file=$1
  local ext=$2
  local ros_pkg=$3
  local ros_out=
  local my_file_tmp=
  found_it=true

  if [[ ${ext} = "msg" ]] || [[ ${ext} = "srv" ]]
  then
    my_file_tmp="${my_file%.*}"
    ros_out=`eval "ros${ext} list | grep ${my_file_tmp}"`
    #ros_out=`eval "ros${ext} show ${my_file}"`
  else
    #action file
    ros_out=`eval "rosmsg show ${my_file}Action"`

    local act_msgs="${my_file}Action ${my_file}ActionFeedback ${my_file}ActionGoal ${my_file}ActionResult ${my_file}Feedback ${my_file}Goal ${my_file}Result"
    for am in ${act_msgs}
    do
      ros_out=`eval "rosmsg list | grep ${am}"`
      #ros_out=`eval "rosmsg show ${am}"`
      if [[ -z "${ros_out}" ]]
      then
        found_it=false
        break
      fi
    done
  fi

  if [[ -n "${ros_out}" ]]
  then
    found_it=true

    if [[ ${my_file} == */* ]]
    then
      local parsed_name=$(echo ${my_file} | sed 's/\// /g')
      file_pkg=${parsed_name%% *}
      my_file=${parsed_name##* }

    else
      file_pkg=
    fi

    if [[ -z ${file_pkg} ]]
    then
      file_pkg=(`echo ${ros_out} | tr '[' ' ' | tr ']' ' ' | tr '/' ' '`)
    fi

    echo "ros_pkg=${ros_pkg}"
    echo "file_pkg=${file_pkg}"
    #check if both packages are the same
    if [[ ${ros_pkg} != ${file_pkg} ]]
    then
      #modify package.xml to add dependency
      add_pkg_to_packagexml ${ros_pkg} ${file_pkg} 
    fi

  else
    found_it=false
  fi
}

# add_pkg_to_packagexml
# Adds $2 as dependency to $1 package.xml file.
# - $1: current ros package
# - $2: message ros package
function add_pkg_to_packagexml
{
  if [[ -z $1 ]] || [[ -z $2 ]]
  then
    kill_exit "add_pkg_to_packagexml: missing input parameters"
  fi

  #get path where message is stored
  local ros_pkg=$1
  local file_pkg=$2

  #go to package folder
  roscd "${ros_pkg}"

  #look for package in package.xml and add dependency if necessary
  local pub="<build_depend>${file_pkg}<build_depend/>"
  local pub1=$(grep "${pub}" "package.xml")
  if [[ -z "${pub1}" ]]
  then
    echo "Adding package \"${file_pkg}\" as build dependency..."
    local comment="\"base_"
    pub="<build_depend>${file_pkg}<build_depend/>"
    sed -i -e "/${comment}/a\\  ${pub}" "package.xml"
  else
    echo "Package \"${file_pkg}\" already added as build dependency, skipping"
  fi

  local pub="<build_export_depend>${file_pkg}<build_export_depend/>"
  local pub1=$(grep "${pub}" "package.xml")
  if [[ -z "${pub1}" ]]
  then
    echo "Adding package \"${file_pkg}\" as build export dependency..."
    local comment="\"base_"
    pub="<build_export_depend>${file_pkg}<build_export_depend/>"
    sed -i -e "/${comment}/a\\  ${pub}" "package.xml"
  else
    echo "Package \"${file_pkg}\" already added as build export dependency, skipping"
  fi
}

# find_comment_in_file
# Searches for $1 in $2. Returns true if found.
# - $1: line to be found in file
# - $2: text file name
function find_comment_in_file
{
  if [[ -z $1 ]] || [[ -z $2 ]]
  then
    kill_exit "find_comment_in_file: missing input parameters"
  fi

  local comment_to_find=$1
  local file=$2
  comment_found="true"

  #look for key comment
  local comm_found=$(grep -n "${comment_to_find}" "${file}")
  if [[ -z "${comm_found}" ]]
  then
    comment_found="false"
  else
    line_number=${comm_found%%:*}
  fi
}

# add_line_to_file
# Looks for comment line $2 in $3 file. If found, places line $1 after comment.
# If comment is not found, exits script. If $1 already exists, skips adding it 
# again.
# - $1: line to be added
# - $2: comment to be found
# - $3: text file name
function add_line_to_file
{
  if [[ -z $1 ]] || [[ -z $2 ]] || [[ -z $3 ]]
  then
    kill_exit "add_line_to_file: missing input parameters"
  fi

  local line_to_add=$1
  local comment_to_find=$2
  local file=$3

  #look for key comment
  find_comment_in_file "${comment_to_find}" "${file}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "File ${file} needs to be restored, missing comment: \"${comment_to_find}\""
  fi

  #check if line already exists in file
  local line_found=$(grep "${line_to_add}" "${file}")

  if [[ -z "${line_found}" ]]
  then
    #add line to file
#    echo "Modifying file ${file} with:"
#    echo "     ${line_to_add}"
    sed -i -e "/${comment}/a\\${line_to_add}" "${file}"
  else
    #skip line
    echo "File ${file} already included the line, skipping..."
  fi
}

# get_h_cpp_files
# Looks for header files in include folder, if empty looks for headers files
# in include/pkg_name folder.
# - $1: pkg name
function get_h_cpp_files
{
  if [[ -z $1 ]]
  then
    kill_exit "get_h_cpp_files: missing input parameters"
  fi

  local ros_pkg=$1
  node_h=""
  node_c=""
  roscd "${ros_pkg}"

  if [[ -z `ls -l "include/" | grep "^-" | grep "\.h$"` ]]
  then
    if [[ -z `ls -l "include/${ros_pkg}" | grep "^-" | grep "\.h$"` ]]
    then
      kill_exit "no include files found!"
    else
      node_h="include/${ros_pkg}/${ros_pkg}.h"
      node_c="src/${ros_pkg}.cpp"
    fi
  else
    kill_exit "this node does not have the folder structure \"include/pkg_name/*.h\"!"
  fi
}

function change_license_to_LGPL
{
      find . -name package.xml -exec sed -i -e 's:<license>.*</license>:<license>LGPL</license>:g' {} \;
}

function goto_catkin_workspace
{
  roscd
  if [[ -f .catkin ]]
  then
    cd .. # to catkin work space
  else
    setup_line=$(grep "setup-file" ".rosinstall")
    ros_path=$(sed -n 's/.*- setup-file: {local-name: \(.*\)devel.*/\1/p' <<< ${setup_line})
    cd ${ros_path}
  fi
}

function check_cmakelists_file_integrity
{
  local comment=""

  comment="add_dependencies(\${PROJECT_NAME} <msg_package_name>_generate_messages_cpp)"
  find_comment_in_file "${comment}" "CMakeLists.txt"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing add_dependencies(\${PROJECT_NAME} <msg_package_name>_generate_messages_cpp) from the CMakeLists.txt file"
  fi
}

function add_build_run_dependencies
{
  local ros_pkg=$1
  local new_pkg=$2
  local line=""
  local comment=""

  if [[ "${new_pkg}" != "${ros_pkg}" ]]
  then

    line="<exec_depend>${new_pkg}<\/exec_depend>"
    find_comment_in_file "${line}" "package.xml"
    if [[ "${comment_found}" = "false" ]]
    then
      line="<exec_depend>${new_pkg}<\/exec_depend>"
      comment="<exec_depend>roslib<\/exec_depend>"
      add_line_to_file "\ \ ${line}" "${comment}" "package.xml"
    else
      echo "Build dependencies already included."
    fi


    line="<build_depend>${new_pkg}<\/build_depend>"
    find_comment_in_file "${line}" "package.xml"
    if [[ "${comment_found}" = "false" ]]
    then
      line="<build_depend>${new_pkg}<\/build_depend>"
      comment="<build_depend>roslib<\/build_depend>"
      add_line_to_file "\ \ ${line}" "${comment}" "package.xml"
    else
      echo "Build dependencies already included."
    fi

    line="<build__export_depend>${new_pkg}<\/build_export_depend>"
    find_comment_in_file "${line}" "package.xml"
    if [[ "${comment_found}" = "false" ]]
    then
      line="<build_export_depend>${new_pkg}<\/build_export_depend>"
      comment="<build_export_depend>roslib<\/build_export_depend>"
      add_line_to_file "\ \ ${line}" "${comment}" "package.xml"
    else
      echo "Build dependencies already included."
    fi
  

#    line="<run_depend>${new_pkg}<\/run_depend>"
#    find_comment_in_file "${line}" "package.xml"
#    if [[ "${comment_found}" = "false" ]]
#    then
#      line="<run_depend>${new_pkg}<\/run_depend>"
#      comment="<run_depend>roslib<\/run_depend>"
#      add_line_to_file "\ \ ${line}" "${comment}" "package.xml"
#    else
#      echo "Run dependencies already included."
#    fi
  fi
}

function add_cmake_dependencies
{
  local ros_pkg=$1
  local new_pkg=$2
  local line=""
  local old_line=""
  local comment=""
  local old_string=""
  local new_string=""

  if [[ "${new_pkg}" != "${ros_pkg}" ]]
  then
    old_line=$(grep "find_package(catkin REQUIRED COMPONENTS" "CMakeLists.txt")
    if [[ $old_line == *${new_pkg}* ]]
    then
      echo "Dependency already included in CMakeLists.txt file.";
    else
      old_string=" dynamic_reconfigure"
      new_string="${old_string}\ ${new_pkg}"
      sed -i "s/${old_string}/${new_string}/g" "CMakeLists.txt"
    fi

    # check if dependency has messages to be build previously
    roscd "${new_pkg}"
    if [ -d "msg" ]
    then
      roscd "${ros_pkg}"
      line="add_dependencies(\${PROJECT_NAME} ${new_pkg}_generate_messages_cpp)"
      comment="add_dependencies(\${PROJECT_NAME} <msg_package_name>_generate_messages_cpp)"
      add_line_to_file "${line}" "${comment}" "CMakeLists.txt"
    fi
    roscd "${ros_pkg}"
  fi
}

# 11. Bash File Testing
# -b filename   Block special file
# -c filename   Special character file
# -d directoryname  Check for directory existence
# -e filename   Check for file existence
# -f filename   Check for regular file existence not a directory
# -G filename   Check if file exists and is owned by effective group ID.
# -g filename   true if file exists and is set-group-id.
# -k filename   Sticky bit
# -L filename   Symbolic link
# -O filename   True if file exists and is owned by the effective user id.
# -r filename   Check if file is a readable
# -S filename   Check if file is socket
# -s filename   Check if file is nonzero size
# -u filename   Check if file set-ser-id bit is set
# -w filename   Check if file is writable
# -x filename   Check if file is executable
# -n variable   Check if variable is not null (contains one or more characaters)
# -z variable   Check if variable is null (empty)
