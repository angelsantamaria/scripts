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

function check_action_client_file_integrity
{
  local node_h=$1
  local node_c=$2
  local comment=""

  # check node.h file
  comment="\[action server client headers\]"
  find_comment_in_file "${comment}" "${node_h}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[action server client headers\] from the header file ${node_h}"
  fi
  comment="\[action client attributes\]"
  find_comment_in_file "${comment}" "${node_h}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[action client attributes\] from the header file ${node_h}"
  fi

  # check node.cpp file
  comment="\[fill action structure and make request to the action server\]"
  find_comment_in_file "${comment}" "${node_c}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[fill action structure and make request to the action server\] from the source file ${node_c}"
  fi

  comment="\[action callbacks\]"
  find_comment_in_file "${comment}" "${node_c}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[action callbacks\] from the source file ${node_c}"
  fi

  comment="\[action requests\]"
  find_comment_in_file "${comment}" "${node_c}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[action requests\] from the source file ${node_c}"
  fi
}

function check_action_client_attributes_functions
{
  local action_file=$1
  local act_pkg=$2
  local node_h=$3
  local node_c=$4
  local client_name=$5
  local act_goal=$6
  local topic_name=$7
  local class_name=$8
  local aux_line=""

  # check the node.h file
  aux_line="actionlib::SimpleActionClient<${act_pkg}::${action_file}Action> ${client_name};"
  find_comment_in_file "${aux_line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "An ActionClient variable already exists with the same name in file ${node_h} line ${line_number}"
  fi

  aux_line="${act_pkg}::${action_file}Goal ${act_goal};"
  find_comment_in_file "${aux_line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "An ActionClient goal message variable already exists with the same name in file ${node_h} line ${line_number}"
  fi

  aux_line="bool ${topic_name}MakeActionRequest();"
  find_comment_in_file "${aux_line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A MakeActionRequest function already exists with the same name in file ${node_h} line ${line_number}"
  fi

  aux_line="void ${topic_name}Done(const actionlib::SimpleClientGoalState& state,  const ${act_pkg}::${action_file}ResultConstPtr& result);"
  find_comment_in_file "${aux_line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A Done callback function already exists with the same name in file ${node_h} line ${line_number}"
  fi

  aux_line="void ${topic_name}Active();"
  find_comment_in_file "${aux_line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "An Active callback function already exists with the same name in file ${node_h} line ${line_number}"
  fi

  aux_line="void ${topic_name}Feedback(const ${act_pkg}::${action_file}FeedbackConstPtr& feedback);"
  find_comment_in_file "${aux_line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A Feedback callback function already exists with the same name in file ${node_h} line ${line_number}"
  fi

  # check the node.c file
  aux_line="${client_name}(\"${topic_name}\", true)"
  find_comment_in_file "${aux_line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "The constructor for the Action client is already called in file ${node_c} line ${line_number}"
  fi

  aux_line="void ${class_name}::${topic_name}Done(const actionlib::SimpleClientGoalState& state,  const ${act_pkg}::${action_file}ResultConstPtr& result)"
  find_comment_in_file "${aux_line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "An implementation for the Done callback function is already present in file ${node_c} line ${line_number}"
  fi

  aux_line="void ${class_name}::${topic_name}Active()"
  find_comment_in_file "${aux_line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "An implementation for the Active callback function is already present in file ${node_c} line ${line_number}"
  fi

  aux_line="void ${class_name}::${topic_name}Feedback(const ${act_pkg}::${action_file}FeedbackConstPtr& feedback)"
  find_comment_in_file "${aux_line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "An implementation for the Feedback callback function is already present in file ${node_c} line ${line_number}"
  fi

  aux_line="bool ${class_name}::${topic_name}MakeActionRequest()"
  find_comment_in_file "${aux_line}" "${node_c}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "An implementation for the MakeActionRequest function is already present in file ${node_c} line ${line_number}"
  fi
}

function create_action_client
{
  #read input params
  local ros_pkg=$1
  local topic_name=$2
  local action_file=$3
  local act_pkg=$4
  local node_h=$5
  local node_c=$6
  local driver_alg=$7
  local client_name="${topic_name}_client_"
  local act_goal="${topic_name}_goal_"
  local line=""
  local old_line=""
  local comment=""
  local old_string=""
  local new_string=""
  local add_char=""

  #get class basename
  get_class_basename "${node_c}"
  if [[ -z ${class_name} ]]
  then
    kill_exit "impossible to retrieve class basename"
  fi
  # echo "class_name=${class_name}"

  #check files integrity before making any changes
  check_cmakelists_file_integrity "${driver_alg}"
  check_action_client_file_integrity "${node_h}" "${node_c}"
  check_action_client_attributes_functions "${action_file}" "${act_pkg}" "${node_h}" "${node_c}" "${client_name}" "${act_goal}" "${topic_name}" "${class_name}"

################################################################################
#modify package.xml

  local actionlib_pkg="actionlib"

  add_build_run_dependencies "${ros_pkg}" "${actionlib_pkg}"
  add_build_run_dependencies "${ros_pkg}" "${act_pkg}"

###############################################################################
#modify CMakeLists.txt

  old_line=$(grep "find_package(catkin REQUIRED COMPONENTS" "CMakeLists.txt")
  if [[ $old_line == *${actionlib_pkg}* ]]
  then
    echo "Dependency already included in CMakeLists.txt file.";
  else
    old_string=" dynamic_reconfigure"
    new_string="${old_string}\ ${actionlib_pkg}"
    sed -i "s/${old_string}/${new_string}/g" "CMakeLists.txt"
  fi

  add_cmake_dependencies "${ros_pkg}" "${act_pkg}"

################################################################################
#modify Node.h

  #look for include files and add them if are not already there
  line="#include <${act_pkg}/${action_file}Action.h>"
  comment="\[action server client headers\]"
  add_line_to_file "${line}" "${comment}" "${node_h}"

  #look for include files and add them if are not already there
  line="#include <actionlib/client/terminal_state.h>"
  comment="\[action server client headers\]"
  add_line_to_file "${line}" "${comment}" "${node_h}"

  #look for include files and add them if are not already there
  line="#include <actionlib/client/simple_action_client.h>"
  comment="\[action server client headers\]"
  add_line_to_file "${line}" "${comment}" "${node_h}"

  aux_line="actionlib::SimpleActionClient<${act_pkg}::${action_file}Action> ${client_name};"
  line="\ \ \ \ ${aux_line}\n"
  aux_line="${act_pkg}::${action_file}Goal ${act_goal};"
  line="${line}\ \ \ \ ${aux_line}\n"
  aux_line="bool ${topic_name}MakeActionRequest();"
  line="${line}\ \ \ \ ${aux_line}\n"
  aux_line="void ${topic_name}Done(const actionlib::SimpleClientGoalState& state,  const ${act_pkg}::${action_file}ResultConstPtr& result);"
  line="${line}\ \ \ \ ${aux_line}\n"
  aux_line="void ${topic_name}Active();"
  line="${line}\ \ \ \ ${aux_line}\n"
  aux_line="void ${topic_name}Feedback(const ${act_pkg}::${action_file}FeedbackConstPtr& feedback);"
  line="${line}\ \ \ \ ${aux_line}\n"

  comment="\[action client attributes\]"
  add_line_to_file "${line}" "${comment}" "${node_h}"

################################################################################
#modify Node.cpp

  line="${client_name}(\"${topic_name}\", true)"
  #   comment="${class_name}::${class_name}("
  #   sed -i "/${comment}/s|$|,|" "${node_c}"
  #   add_line_to_file "\ \ ${line}" "${comment}" "${node_c}"

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

  aux_line="\ \ // IMPORTANT: Please note that all mutex used in the client callback functions"
  line="${aux_line}"
  find_comment_in_file "${line}" "${node_c}"
  if [[ "${comment_found}" = "false" ]]
  then
    aux_line="\ \ // must be unlocked before calling any of the client class functions from an"
    line="${line}\n${aux_line}\n"
    aux_line="\ \ // other thread (MainNodeThread)."
    line="${line}${aux_line}\n"
    comment="\[fill action structure and make request to the action server\]"
    add_line_to_file "${line}" "${comment}" "${node_c}"
  fi

  aux_line="\ \ // variable to hold the state of the current goal on the server"
  line="${aux_line}\n"
  aux_line="\ \ //actionlib::SimpleClientGoalState ${topic_name}_state(actionlib::SimpleClientGoalState::PENDING);"
  line="${line}${aux_line}\n"
  aux_line="\ \ // to get the state of the current goal"
  line="${line}${aux_line}\n"
  aux_line="\ \ //${driver_alg}_.unlock();" 
  line="${line}${aux_line}\n"
  aux_line="\ \ //${topic_name}_state=${topic_name}_client_.getState();"
  line="${line}${aux_line}\n"
  aux_line="\ \ // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST"
  line="${line}${aux_line}\n"
  aux_line="\ \ //${driver_alg}_.lock();"
  line="${line}${aux_line}\n"
  aux_line="\ \ //if(${topic_name}_state==actionlib::SimpleClientGoalState::ABORTED)"
  line="${line}${aux_line}\n"
  aux_line="\ \ //{"
  line="${line}${aux_line}\n"
  aux_line="\ \ //  do something"
  line="${line}${aux_line}\n"
  aux_line="\ \ //}"
  line="${line}${aux_line}\n"
  aux_line="\ \ //else if(${topic_name}_state==actionlib::SimpleClientGoalState::SUCCEEDED)"
  line="${line}${aux_line}\n"
  aux_line="\ \ //{"
  line="${line}${aux_line}\n"
  aux_line="\ \ //  do something else"
  line="${line}${aux_line}\n"
  aux_line="\ \ //}"
  line="${line}${aux_line}\n"
  aux_line="\ \ //${topic_name}MakeActionRequest();"
  line="${line}${aux_line}\n"
  comment="\[fill action structure and make request to the action server\]"
  add_line_to_file "${line}" "${comment}" "${node_c}"

  aux_line="void ${class_name}::${topic_name}Done(const actionlib::SimpleClientGoalState& state,  const ${act_pkg}::${action_file}ResultConstPtr& result)"
  line="${aux_line}\n"
  aux_line="{"
  line="${line}${aux_line}\n"
  aux_line="\ \ ${driver_alg}_.lock();"
  line="${line}${aux_line}\n"
  aux_line="\ \ if( state == actionlib::SimpleClientGoalState::SUCCEEDED )"
  line="${line}${aux_line}\n"
  aux_line="\ \ \ \ ROS_INFO(\"${class_name}::${topic_name}Done: Goal Achieved!\");"
  line="${line}${aux_line}\n"
  aux_line="\ \ else"
  line="${line}${aux_line}\n"
  aux_line="\ \ \ \ ROS_INFO(\"${class_name}::${topic_name}Done: %s\", state.toString().c_str());"
  line="${line}${aux_line}\n\n"
  aux_line="\ \ //copy & work with requested result"
  line="${line}${aux_line}\n"
  aux_line="\ \ ${driver_alg}_.unlock();"
  line="${line}${aux_line}\n"
  aux_line="}"
  line="${line}${aux_line}\n\n"
  aux_line="void ${class_name}::${topic_name}Active()"
  line="${line}${aux_line}\n"
  aux_line="{"
  line="${line}${aux_line}\n"
  aux_line="\ \ ${driver_alg}_.lock();"
  line="${line}${aux_line}\n"
  aux_line="\ \ //ROS_INFO(\"${class_name}::${topic_name}Active: Goal just went active!\");"
  line="${line}${aux_line}\n"
  aux_line="\ \ ${driver_alg}_.unlock();"
  line="${line}${aux_line}\n"
  aux_line="}"
  line="${line}${aux_line}\n\n"
  aux_line="void ${class_name}::${topic_name}Feedback(const ${act_pkg}::${action_file}FeedbackConstPtr& feedback)"
  line="${line}${aux_line}\n"
  aux_line="{"
  line="${line}${aux_line}\n"
  aux_line="\ \ ${driver_alg}_.lock();"
  line="${line}${aux_line}\n"
  aux_line="\ \ //ROS_INFO(\"${class_name}::${topic_name}Feedback: Got Feedback!\");"
  line="${line}${aux_line}\n\n"
  aux_line="\ \ bool feedback_is_ok = true;"
  line="${line}${aux_line}\n\n"
  aux_line="\ \ //analyze feedback"
  line="${line}${aux_line}\n"
  aux_line="\ \ //my_var = feedback->var;"
  line="${line}${aux_line}\n\n"
  aux_line="\ \ //if feedback is not what expected, cancel requested goal"
  line="${line}${aux_line}\n"
  aux_line="\ \ if( !feedback_is_ok )"
  line="${line}${aux_line}\n"
  aux_line="\ \ {"
  line="${line}${aux_line}\n"
  aux_line="\ \ \ \ ${client_name}.cancelGoal();"
  line="${line}${aux_line}\n"
  aux_line="\ \ \ \ //ROS_INFO(\"${class_name}::${topic_name}Feedback: Cancelling Action!\");"
  line="${line}${aux_line}\n"
  aux_line="\ \ }"
  line="${line}${aux_line}\n"
  aux_line="\ \ ${driver_alg}_.unlock();"
  line="${line}${aux_line}\n"
  aux_line="}"
  line="${line}${aux_line}\n"
  comment="\[action callbacks\]"
  add_line_to_file "${line}" "${comment}" "${node_c}"

  aux_line="bool ${class_name}::${topic_name}MakeActionRequest()"
  line="${aux_line}\n"
  aux_line="{"
  line="${line}${aux_line}\n"
  aux_line="\ \ // IMPORTANT: Please note that all mutex used in the client callback functions"
  line="${line}${aux_line}\n"
  aux_line="\ \ // must be unlocked before calling any of the client class functions from an"
  line="${line}${aux_line}\n"
  aux_line="\ \ // other thread (MainNodeThread)."
  line="${line}${aux_line}\n"
  aux_line="\ \ // this->${driver_alg}_.unlock();"
  line="${line}${aux_line}\n"
  aux_line="\ \ if(${client_name}.isServerConnected())"
  line="${line}${aux_line}\n"
  aux_line="\ \ {"
  line="${line}${aux_line}\n"
  aux_line="\ \ \ \ //ROS_DEBUG(\"${class_name}::${topic_name}MakeActionRequest: Server is Available!\");"
  line="${line}${aux_line}\n"
  aux_line="\ \ \ \ //send a goal to the action server"
  line="${line}${aux_line}\n"
  aux_line="\ \ \ \ //${act_goal}.data = my_desired_goal;" 
  line="${line}${aux_line}\n"
  aux_line="\ \ \ \ ${client_name}.sendGoal(${act_goal},"
  line="${line}${aux_line}\n"
  aux_line="\ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ boost::bind(&${class_name}::${topic_name}Done,     this, _1, _2),"
  line="${line}${aux_line}\n"
  aux_line="\ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ boost::bind(&${class_name}::${topic_name}Active,   this),"
  line="${line}${aux_line}\n"
  aux_line="\ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ boost::bind(&${class_name}::${topic_name}Feedback, this, _1));"
  line="${line}${aux_line}\n"
  aux_line="\ \ \ \ // this->${driver_alg}_.lock();"
  line="${line}${aux_line}\n"
  aux_line="\ \ \ \ // ROS_DEBUG(\"${class_name}::${topipc_name}MakeActionRequest: Goal Sent.\");"
  line="${line}${aux_line}\n"
  aux_line="\ \ \ \ return true;"
  line="${line}${aux_line}\n"
  aux_line="\ \ }"
  line="${line}${aux_line}\n"
  aux_line="\ \ else"
  line="${line}${aux_line}\n"
  aux_line="\ \ {"
  line="${line}${aux_line}\n"
  aux_line="\ \ \ \ // this->${driver_alg}_.lock();"
  line="${line}${aux_line}\n"
  aux_line="\ \ \ \ // ROS_DEBUG(\"${class_name}::${topic_name}MakeActionRequest: HRI server is not connected\");"
  line="${line}${aux_line}\n"
  aux_line="\ \ \ \ return false;"
  line="${line}${aux_line}\n"
  aux_line="\ \ }"
  line="${line}${aux_line}\n"
  aux_line="}"
  line="${line}${aux_line}\n"
  comment="\[action requests\]"
  add_line_to_file "${line}" "${comment}" "${node_c}"

# ################################################################################
#compile
  goto_catkin_workspace
  #catkin_make --only-pkg-with-deps ${ros_pkg}
  catkin build ${ros_pkg}
}
