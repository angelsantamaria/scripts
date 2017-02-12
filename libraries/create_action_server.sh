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

function check_action_server_files_integrity
{
  local node_h=$1
  local node_c=$2
  local comment=""

  # check the node.h file
  comment="\[action server client headers\]"
  find_comment_in_file "${comment}" "${node_h}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[action server client headers\] from the header file ${node_h}"
  fi
  comment="\[action server attributes\]" 
  find_comment_in_file "${comment}" "${node_h}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[action server attributes\] from the header file ${node_h}"
  fi

  # check the node.cpp file
  comment="\[init action servers\]"
  find_comment_in_file "${comment}" "${node_c}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[init action servers\] from the source file ${node_c}"
  fi
  comment="\[action callbacks\]"
  find_comment_in_file "${comment}" "${node_c}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[action callbacks\] from the source file ${node_c}"
  fi
  comment="\[fill action structure and make request to the action server\]"
  find_comment_in_file "${comment}" "${node_c}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[fill action structure and make request to the action server\] from the source file ${node_c}"
  fi
}

function check_action_server_attributes_functions
{
  local action_file=$1
  local act_pkg=$2
  local node_h=$3
  local node_c=$4
  local server_name=$5
  local act_goal=$6
  local topic_name=$7
  local class_name=$8
  local act_start=$9
  local act_stop=${10}
  local act_finish=${11}
  local act_succeed=${12}
  local act_result=${13}
  local act_feedback=${14}

  local aux_line=""

  # check the node.h file
  aux_line="ActionServer<${act_pkg}::${action_file}Action> ${server_name};"
  find_comment_in_file "${aux_line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "An ActionServer variable already exists with the same name in file ${node_h} line ${line_number}"
  fi

  aux_line="void ${act_start}(const ${act_pkg}::${action_file}GoalConstPtr& goal);"
  find_comment_in_file "${aux_line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A start callback function already exists with the same name in file ${node_h} line ${line_number}"
  fi

  aux_line="void ${act_stop}(void);"
  find_comment_in_file "${aux_line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A stop callback function already exists with the same name in file ${node_h} line ${line_number}"
  fi

  aux_line="bool ${act_finish}(void);"
  find_comment_in_file "${aux_line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "An is finished callback function already exists with the same name in file ${node_h} line ${line_number}"
  fi

  aux_line="bool ${act_succeed}(void);"
  find_comment_in_file "${aux_line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A has succeeded callback function already exists with the same name in file ${node_h} line ${line_number}"
  fi

  aux_line="void ${act_result}(${act_pkg}::${action_file}ResultPtr& result);"
  find_comment_in_file "${aux_line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A get result callback function already exists with the same name in file ${node_h} line ${line_number}"
  fi

  aux_line="void ${act_feedback}(${act_pkg}::${action_file}FeedbackPtr& feedback);"
  find_comment_in_file "${aux_line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A get feedback callback function already exists with the same name in file ${node_h} line ${line_number}"
  fi
  aux_line="bool ${topic_name}_active;"
  find_comment_in_file "${aux_line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "An active boolean already exists with the same name in file ${node_h} line ${line_number}"
  fi
  aux_line="bool ${topic_name}_succeeded;"
  find_comment_in_file "${aux_line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A success boolean already exists with the same name in file ${node_h} line ${line_number}"
  fi
  aux_line="bool ${topic_name}_finished;"
  find_comment_in_file "${aux_line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A finished boolean already exists with the same name in file ${node_h} line ${line_number}"
  fi

  # check the node.cpp file
  aux_line="${server_name}(nh, \"${topic_name}\")"
  find_comment_in_file "${aux_line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "The constructor for the Action server is already called in file ${node_c} line ${line_number}" 
  fi

  aux_line="${server_name}.registerStartCallback"
  find_comment_in_file "${aux_line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "The registration of the start callback function is already present in file ${node_c} line ${line_number}" 
  fi

  aux_line="${server_name}.registerStopCallback"
  find_comment_in_file "${aux_line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "The registration of the stop callback function is already present in file ${node_c} line ${line_number}" 
  fi

  aux_line="${server_name}.registerIsFinishedCallback"
  find_comment_in_file "${aux_line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "The registration of the is finished callback function is already present in file ${node_c} line ${line_number}" 
  fi

  aux_line="${server_name}.registerHasSucceedCallback"
  find_comment_in_file "${aux_line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "The registration of the has succeeded callback function is already present in file ${node_c} line ${line_number}" 
  fi

  aux_line="${server_name}.registerGetResultCallback"
  find_comment_in_file "${aux_line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "The registration of the get result callback function is already present in file ${node_c} line ${line_number}" 
  fi

  aux_line="${server_name}.registerGetFeedbackCallback"
  find_comment_in_file "${aux_line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "The registration of the get feedback callback function is already present in file ${node_c} line ${line_number}" 
  fi

  aux_line="${server_name}.start();"
  find_comment_in_file "${aux_line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "The activation of the action server is already done in file ${node_c} line ${line_number}"
  fi

  line="void ${class_name}::${act_start}(const ${act_pkg}::${action_file}GoalConstPtr& goal)"
  find_comment_in_file "${line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "The implementation of the start callback function is already present in file ${node_c} line ${line_number}" 
  fi

  line="void ${class_name}::${act_stop}(void)"
  find_comment_in_file "${line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "The implementation of the stop callback function is already present in file ${node_c} line ${line_number}" 
  fi

  line="bool ${class_name}::${act_finish}(void)"
  find_comment_in_file "${line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "The implementation of the is finished callback function is already present in file ${node_c} line ${line_number}" 
  fi

  line="bool ${class_name}::${act_succeed}(void)"
  find_comment_in_file "${line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "The implementation of the has succeeded callback function is already present in file ${node_c} line ${line_number}" 
  fi

  line="void ${class_name}::${act_result}(${act_pkg}::${action_file}ResultPtr& result)"
  find_comment_in_file "${line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "The implementation of the get result callback function is already present in file ${node_c} line ${line_number}" 
  fi

  line="void ${class_name}::${act_feedback}(${act_pkg}::${action_file}FeedbackPtr& feedback)"
  find_comment_in_file "${line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "The implementation of the get feedback callback function is already present in file ${node_c} line ${line_number}" 
  fi

  aux_line="this->${topic_name}_active=false;"
  find_comment_in_file "${aux_line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "The active variable of the action server is already initialized in file ${node_c} line ${line_number}"
  fi

  aux_line="this->${topic_name}_succeeded=false;"
  find_comment_in_file "${aux_line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "The success variable of the action server is already initialized in file ${node_c} line ${line_number}"
  fi

  aux_line="this->${topic_name}_finished=false;"
  find_comment_in_file "${aux_line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "The finished variable of the action server is already initialized in file ${node_c} line ${line_number}"
  fi

}

function create_action_server
{
  #read input params
  local ros_pkg=$1
  local topic_name=$2
  local action_file=$3
  local act_pkg=$4
  local node_h=$5
  local node_c=$6
  local driver_alg=$7
  local server_name="${topic_name}_aserver_"
  local act_start="${topic_name}StartCallback"
  local act_stop="${topic_name}StopCallback"
  local act_finish="${topic_name}IsFinishedCallback"
  local act_succeed="${topic_name}HasSucceededCallback"
  local act_result="${topic_name}GetResultCallback"
  local act_feedback="${topic_name}GetFeedbackCallback"
  local line=""
  local old_line=""
  local comment=""
  local aux_line=""
  local add_char=""
  local new_string=""
  local old_string=""

  #get class basename
  get_class_basename "${node_c}"
  if [[ -z ${class_name} ]]
  then
    kill_exit "impossible to retrieve class basename"
  fi
  # echo "class_name=${class_name}"

  #check files integrity before making any changes
  check_cmakelists_file_integrity "${driver_alg}"
  check_action_server_files_integrity "${node_h}" "${node_c}"
  check_action_server_attributes_functions "${action_file}" "${act_pkg}" "${node_h}" "${node_c}" "${server_name}" "${act_goal}" "${topic_name}" "${class_name}" "${act_start}" "${act_stop}" "${act_finish}" "${act_succeed}" "${act_result}" "${act_feedback}"


################################################################################
#create action server header

  #create templates folder path name
  temps_folder="${SCRIPTS_PATH}/ros_node_templates/"

  pkg_path=$(pwd)

  #Set the filename and namespace on the template_alg files
  cp ${SCRIPTS_PATH}/ros_node_templates/template_action_server.h ${pkg_path}/include/action_server.h
  echo "Creating action server file..."
################################################################################


################################################################################
#modify Node.h

  #look for include files and add them if are not already there
  line="#include <${act_pkg}/${action_file}Action.h>"
  comment="\[action server client headers\]"
  add_line_to_file "${line}" "${comment}" "${node_h}"

  #look for include files and add them if are not already there
  line="#include <action_server.h>"
  comment="\[action server client headers\]"
  add_line_to_file "${line}" "${comment}" "${node_h}"

  #look for action server files and add them if are not already there
  aux_line="ActionServer<${act_pkg}::${action_file}Action> ${server_name};"
  line="\ \ \ \ ${aux_line}\n"
  aux_line="void ${act_start}(const ${act_pkg}::${action_file}GoalConstPtr& goal);"
  line="${line}\ \ \ \ ${aux_line}\n"
  aux_line="void ${act_stop}(void);"
  line="${line}\ \ \ \ ${aux_line}\n"
  aux_line="bool ${act_finish}(void);"
  line="${line}\ \ \ \ ${aux_line}\n"
  aux_line="bool ${act_succeed}(void);"
  line="${line}\ \ \ \ ${aux_line}\n"
  aux_line="void ${act_result}(${act_pkg}::${action_file}ResultPtr& result);"
  line="${line}\ \ \ \ ${aux_line}\n"
  aux_line="void ${act_feedback}(${act_pkg}::${action_file}FeedbackPtr& feedback);"
  line="${line}\ \ \ \ ${aux_line}\n"
  aux_line="bool ${topic_name}_active;"
  line="${line}\ \ \ \ ${aux_line}\n"
  aux_line="bool ${topic_name}_succeeded;"
  line="${line}\ \ \ \ ${aux_line}\n"
  aux_line="bool ${topic_name}_finished;"
  line="${line}\ \ \ \ ${aux_line}\n"

  comment="\[action server attributes\]"
  add_line_to_file "${line}" "${comment}" "${node_h}"

################################################################################
#modify Node.cpp

  line="${server_name}(nh, \"${topic_name}\")"

  #check if ':' are needed
  comment="${class_name}::${class_name}("
  add_char=`grep "${comment}" "${node_c}" | grep " :"`
  if [[ -z ${add_char} ]]
  then
    sed -i "/${comment}/s|$| :|" "${node_c}"
    add_line_to_file "\ \ ${line}" "${comment}" "${node_c}"
  else
    #check if ',' is needed
    add_char=`grep "${comment}" "${node_c}" | grep ","`
    if [[ -z ${add_char} ]]
    then
      #add ',' to comment
      sed -i "/${comment}/s|$|,|" "${node_c}"
      add_line_to_file "\ \ ${line}" "${comment}" "${node_c}"
    else
      add_line_to_file "\ \ ${line}," "${comment}" "${node_c}"
    fi
  fi

  aux_line="\ \ // IMPORTANT: it is better to use the boolean variables to control the"
  line="${aux_line}\n"
  find_comment_in_file "${line}" "${node_c}"
  if [[ "${comment_found}" = "false" ]]
  then
    aux_line="\ \ // behavior of the action server instead of direclty calling the action server"
    line="${line}${aux_line}\n"
    aux_line="\ \ // class functions."
    line="${line}${aux_line}\n"
    comment="\[fill action structure and make request to the action server\]"
    add_line_to_file "${line}" "${comment}" "${node_c}"
  fi

  aux_line="\ \ // To finish the action server with success"
  line="${aux_line}\n"
  aux_line="\ \ //this->${topic_name}_succeeded=true;"
  line="${line}${aux_line}\n"
  aux_line="\ \ //this->${topic_name}_finished=true;"
  line="${line}${aux_line}\n"
  aux_line="\ \ // To finish the action server with failure"
  line="${line}${aux_line}\n"
  aux_line="\ \ //this->${topic_name}_succeeded=false;"
  line="${line}${aux_line}\n"
  aux_line="\ \ //this->${topic_name}_finished=true;"
  line="${line}${aux_line}\n"
  comment="\[fill action structure and make request to the action server\]"
  add_line_to_file "${line}" "${comment}" "${node_c}"

  aux_line="${server_name}.registerStartCallback(boost::bind(&${class_name}::${act_start}, this, _1));"
  line="${aux_line}\n"
  aux_line="\ \ ${server_name}.registerStopCallback(boost::bind(&${class_name}::${act_stop}, this));"
  line="${line}${aux_line}\n"
  aux_line="\ \ ${server_name}.registerIsFinishedCallback(boost::bind(&${class_name}::${act_finish}, this));"
  line="${line}${aux_line}\n"
  aux_line="\ \ ${server_name}.registerHasSucceedCallback(boost::bind(&${class_name}::${act_succeed}, this));"
  line="${line}${aux_line}\n"
  aux_line="\ \ ${server_name}.registerGetResultCallback(boost::bind(&${class_name}::${act_result}, this, _1));"
  line="${line}${aux_line}\n"
  aux_line="\ \ ${server_name}.registerGetFeedbackCallback(boost::bind(&${class_name}::${act_feedback}, this, _1));"
  line="${line}${aux_line}\n"
  aux_line="\ \ ${server_name}.start();"
  line="${line}${aux_line}\n"
  aux_line="\ \ this->${topic_name}_active=false;"
  line="${line}${aux_line}\n"
  aux_line="\ \ this->${topic_name}_succeeded=false;"
  line="${line}${aux_line}\n"
  aux_line="\ \ this->${topic_name}_finished=false;"
  line="${line}${aux_line}\n"
  comment="\[init action servers\]"
  add_line_to_file "\ \ ${line}" "${comment}" "${node_c}"

  aux_line="void ${class_name}::${act_start}(const ${act_pkg}::${action_file}GoalConstPtr& goal)"
  line="${aux_line}\n"
  aux_line="{"
  line="${line}${aux_line}\n"
  aux_line="\ \ this->${driver_alg}_.lock();"
  line="${line}${aux_line}\n"
  aux_line="\ \ //check goal"
  line="${line}${aux_line}\n"
  aux_line="\ \ this->${topic_name}_active=true;"
  line="${line}${aux_line}\n"
  aux_line="\ \ this->${topic_name}_succeeded=false;"
  line="${line}${aux_line}\n"
  aux_line="\ \ this->${topic_name}_finished=false;"
  line="${line}${aux_line}\n"
  aux_line="\ \ //execute goal"
  line="${line}${aux_line}\n"
  aux_line="\ \ this->${driver_alg}_.unlock();"
  line="${line}${aux_line}\n"
  aux_line="}"
  line="${line}${aux_line}\n\n"
  aux_line="void ${class_name}::${act_stop}(void)"
  line="${line}${aux_line}\n"
  aux_line="{"
  line="${line}${aux_line}\n"
  aux_line="\ \ this->${driver_alg}_.lock();"
  line="${line}${aux_line}\n"
  aux_line="\ \ //stop action"
  line="${line}${aux_line}\n"
  aux_line="\ \ this->${topic_name}_active=false;"
  line="${line}${aux_line}\n"
  aux_line="\ \ this->${driver_alg}_.unlock();"
  line="${line}${aux_line}\n"
  aux_line="}"
  line="${line}${aux_line}\n\n"
  aux_line="bool ${class_name}::${act_finish}(void)"
  line="${line}${aux_line}\n"
  aux_line="{"
  line="${line}${aux_line}\n"
  aux_line="\ \ bool ret = false;"
  line="${line}${aux_line}\n\n"
  aux_line="\ \ this->${driver_alg}_.lock();"
  line="${line}${aux_line}\n"
  aux_line="\ \ //if action has finish for any reason"
  line="${line}${aux_line}\n"
  aux_line="\ \ ret = this->${topic_name}_finished;"
  line="${line}${aux_line}\n"
  aux_line="\ \ this->${driver_alg}_.unlock();"
  line="${line}${aux_line}\n\n"
  aux_line="\ \ return ret;"
  line="${line}${aux_line}\n"
  aux_line="}"
  line="${line}${aux_line}\n\n"
  aux_line="bool ${class_name}::${act_succeed}(void)"
  line="${line}${aux_line}\n"
  aux_line="{"
  line="${line}${aux_line}\n"
  aux_line="\ \ bool ret = false;"
  line="${line}${aux_line}\n\n"
  aux_line="\ \ this->${driver_alg}_.lock();"
  line="${line}${aux_line}\n"
  aux_line="\ \ //if goal was accomplished"
  line="${line}${aux_line}\n"
  aux_line="\ \ ret = this->${topic_name}_succeeded;"
  line="${line}${aux_line}\n"
  aux_line="\ \ this->${topic_name}_active=false;"
  line="${line}${aux_line}\n"
  aux_line="\ \ this->${driver_alg}_.unlock();"
  line="${line}${aux_line}\n\n"
  aux_line="\ \ return ret;"
  line="${line}${aux_line}\n"
  aux_line="}"
  line="${line}${aux_line}\n\n"
  aux_line="void ${class_name}::${act_result}(${act_pkg}::${action_file}ResultPtr& result)"
  line="${line}${aux_line}\n"
  aux_line="{"
  line="${line}${aux_line}\n"
  aux_line="\ \ this->${driver_alg}_.lock();"
  line="${line}${aux_line}\n"
  aux_line="\ \ //update result data to be sent to client"
  line="${line}${aux_line}\n"
  aux_line="\ \ //result->data = data;"
  line="${line}${aux_line}\n"
  aux_line="\ \ this->${driver_alg}_.unlock();"
  line="${line}${aux_line}\n"
  aux_line="}"
  line="${line}${aux_line}\n\n"
  aux_line="void ${class_name}::${act_feedback}(${act_pkg}::${action_file}FeedbackPtr& feedback)"
  line="${line}${aux_line}\n"
  aux_line="{"
  line="${line}${aux_line}\n"
  aux_line="\ \ this->${driver_alg}_.lock();"
  line="${line}${aux_line}\n"
  aux_line="\ \ //update feedback data to be sent to client"
  line="${line}${aux_line}\n"
  aux_line="\ \ //ROS_INFO(\"feedback: %s\", feedback->data.c_str());"
  line="${line}${aux_line}\n"
  aux_line="\ \ this->${driver_alg}_.unlock();"
  line="${line}${aux_line}\n"
  aux_line="}"
  line="${line}${aux_line}\n"
  comment="\[action callbacks\]"
  add_line_to_file "${line}" "${comment}" "${node_c}"

################################################################################
# Modify package.xml
# check if the message package is the current ros package

  add_build_run_dependencies "${ros_pkg}" "${act_pkg}"

################################################################################
# modify the CMakeLists.txt file
# check if the message package is the current ros package

  add_cmake_dependencies "${ros_pkg}" "${act_pkg}"

################################################################################
#compile
  goto_catkin_workspace
  catkin_make --only-pkg-with-deps ${ros_pkg}
}
