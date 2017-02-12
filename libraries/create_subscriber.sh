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

function check_subscriber_file_integrity
{
  local node_h=$1
  local node_c=$2
  local comment=""

  # check node.h file
  comment="\[publisher subscriber headers\]"
  find_comment_in_file "${comment}" "${node_h}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[publisher subscriber headers\] from the header file ${node_h}"
  fi
  comment="\[subscriber attributes\]"
  find_comment_in_file "${comment}" "${node_h}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[subscriber attributes\] from the header file ${node_h}"
  fi

  # check node.cpp file
  comment="\[init subscribers\]" 
  find_comment_in_file "${comment}" "${node_c}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[init subscribers\] from the header file ${node_c}"
  fi
  comment="\[subscriber callbacks\]"
  find_comment_in_file "${comment}" "${node_c}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[subscriber callbacks\] from the header file ${node_c}"
  fi
  comment="\[free dynamic memory\]"
  find_comment_in_file "${comment}" "${node_c}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[free dynamic memory\] from the header file ${node_c}"
  fi
}

function check_subscriber_attributes_functions
{
  local node_h=$1
  local node_c=$2
  local subscriber_name=$3
  local callback=$4
  local msg_pkg=$5
  local msg_file=$6
  local mutex_name=$7
  local topic_name=$8
  local class_name=$9
  local buffer=${10}
  local line=""

  # check node.h file
  if [[ "${msg_file}" = "Image" ]]
  then
    line="image_transport::CameraSubscriber ${subscriber_name};"
    find_comment_in_file "${line}" "${node_h}"
    if [[ "${comment_found}" = "true" ]]
    then
      kill_exit "A subscriber with the same name is already declared in file ${node_h} line ${line_number}"
    fi
    line="void ${callback}(const ${msg_pkg}::${msg_file}::ConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& info);"
    find_comment_in_file "${line}" "${node_h}"
    if [[ "${comment_found}" = "true" ]]
    then
      kill_exit "A subscriber callback function with the same name is already declared in file ${node_h} line ${line_number}"
    fi
  else
    line="ros::Subscriber ${subscriber_name};"
    find_comment_in_file "${line}" "${node_h}"
    if [[ "${comment_found}" = "true" ]]
    then
      kill_exit "A subscriber with the same name is already declared in file ${node_h} line ${line_number}"
    fi

    line="void ${callback}(const ${msg_pkg}::${msg_file}::ConstPtr& msg);"
    find_comment_in_file "${line}" "${node_h}"
    if [[ "${comment_found}" = "true" ]]
    then
      kill_exit "A subscriber callback function with the same name is already declared in file ${node_h} line ${line_number}"
    fi
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

  # check node.cpp file
  if [[ "${msg_file}" = "Image" ]]
  then
    line="this->${subscriber_name} = this->it.subscribeCamera(\"${topic_name}/image_raw\", ${buffer}, &${class_name}::${callback}, this);"
    find_comment_in_file "${line}" "${node_c}"
    if [[ "${comment_found}" = "true" ]]
    then
      kill_exit "A subscriber with the same name is already subscribed in file ${node_c} line ${line_number}"
    fi
  else
    line="this->${subscriber_name} = nh.subscribe(\"${topic_name}\", ${buffer}, &${class_name}::${callback}, this);"
    find_comment_in_file "${line}" "${node_c}"
    if [[ "${comment_found}" = "true" ]]
    then
      kill_exit "A subscriber with the same name is already subscribed in file ${node_c} line ${line_number}"
    fi
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

  #look if callback is already defined in file
  if [[ "${msg_file}" = "Image" ]]
  then
    aux_line="void ${class_name}::${callback}(const ${msg_pkg}::${msg_file}::ConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info)"
    find_comment_in_file "${aux_line}" "${node_c}"
    if [[ "${comment_found}" = "true" ]]
    then
      kill_exit "A subscriber callback function with the same name is already implemented in file ${node_c} line ${line_number}"
    fi
  else
    aux_line="void ${class_name}::${callback}(const ${msg_pkg}::${msg_file}::ConstPtr& msg)"
    find_comment_in_file "${aux_line}" "${node_c}"
    if [[ "${comment_found}" = "true" ]]
    then
      kill_exit "A subscriber callback function with the same name is already implemented in file ${node_c} line ${line_number}"
    fi
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

function create_subscriber
{
  #read input params
  local ros_pkg=$1
  local topic_name=$2
  local msg_file=$3
  local msg_pkg=$4
  local buffer=$5
  local node_h=$6
  local node_c=$7
  local driver_alg=$8
  local subscriber_name="${topic_name}_subscriber_"
  local mutex_name="${topic_name}_mutex_"
  local callback="${topic_name}_callback"
  local line=""
  local old_line=""
  local aux_line=""
  local comment=""
  local old_string=""
  local new_string=""

  #get class basename
  get_class_basename "${node_c}"
  if [[ -z ${class_name} ]]
  then
    kill_exit "impossible to retrieve class basename"
  fi
# echo "class_name=${class_name}"

  #check files integrity before making any changes
  check_cmakelists_file_integrity "${driver_alg}"
  check_subscriber_file_integrity "${node_h}" "${node_c}"
  check_subscriber_attributes_functions "${node_h}" "${node_c}" "${subscriber_name}" "${callback}" "${msg_pkg}" "${msg_file}" "${mutex_name}" "${topic_name}" "${class_name}" "${buffer}"

################################################################################
# modify Node.h #

  line="#include <${msg_pkg}/${msg_file}.h>"
  comment="\[publisher subscriber headers\]"
  add_line_to_file "${line}" "${comment}" "${node_h}"
  if [[ "${msg_file}" = "Image" ]]
  then
    line="//#include <cv_bridge/cv_bridge.h>"
    comment="\[publisher subscriber headers\]"
    add_line_to_file "${line}" "${comment}" "${node_h}"
    line="// Uncomment to use the openCV <-> ROS bridge"
    comment="\[publisher subscriber headers\]"
    add_line_to_file "${line}" "${comment}" "${node_h}"
    line="#include <image_transport/image_transport.h>"
    comment="\[publisher subscriber headers\]"
    add_line_to_file "${line}" "${comment}" "${node_h}"
    line="#include <camera_info_manager/camera_info_manager.h>"
    comment="\[publisher subscriber headers\]"
    add_line_to_file "${line}" "${comment}" "${node_h}"
  fi

  if [[ "${msg_file}" = "Image" ]]
  then
    aux_line="image_transport::ImageTransport it;"
    find_comment_in_file "${aux_line}" "${node_h}"
    if [[ "${comment_found}" = "false" ]]
    then
      line="\ \ \ \ ${aux_line}"
      comment="\[subscriber attributes\]"
      add_line_to_file "${line}\n" "${comment}" "${node_h}"
    fi
    aux_line="cv_bridge::CvImagePtr cv_image_;"
    find_comment_in_file "${aux_line}" "${node_h}"
    if [[ "${comment_found}" = "false" ]]
    then
      line="// Uncomment to use the openCV <-> ROS bridge"
      line="\ \ \ \ ${line}\n\ \ \ \ //${aux_line}"
      comment="\[subscriber attributes\]"
      add_line_to_file "${line}" "${comment}" "${node_h}"
    fi
    aux_line="image_transport::CameraSubscriber ${subscriber_name};"
    line="\ \ \ \ ${aux_line}\n"
    aux_line="void ${callback}(const ${msg_pkg}::${msg_file}::ConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& info);"
    line="${line}\ \ \ \ ${aux_line}\n"
  else
    aux_line="ros::Subscriber ${subscriber_name};"
    line="\ \ \ \ ${aux_line}\n"
    aux_line="void ${callback}(const ${msg_pkg}::${msg_file}::ConstPtr& msg);"
    line="${line}\ \ \ \ ${aux_line}\n"
  fi
  aux_line="pthread_mutex_t ${mutex_name};"
  line="${line}\ \ \ \ ${aux_line}\n"
  aux_line="void ${mutex_name}enter(void);"
  line="${line}\ \ \ \ ${aux_line}\n"
  aux_line="void ${mutex_name}exit(void);"
  line="${line}\ \ \ \ ${aux_line}\n"

  comment="\[subscriber attributes\]"
  add_line_to_file "${line}" "${comment}" "${node_h}"

################################################################################
# modify Node.cpp #
  if [[ "${msg_file}" = "Image" ]]
  then
    line="it(nh)"
    find_comment_in_file "${line}" "${node_c}"
    if [[ "${comment_found}" = "false" ]]
    then
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
    fi
    aux_line="this->${subscriber_name} = this->it.subscribeCamera(\"${topic_name}/image_raw\", ${buffer}, &${class_name}::${callback}, this);"
    line="${aux_line}\n"
  else
    aux_line="this->${subscriber_name} = nh.subscribe(\"${topic_name}\", ${buffer}, &${class_name}::${callback}, this);"
    line="${aux_line}\n"
  fi
  aux_line="\ \ pthread_mutex_init(&this->${mutex_name},NULL);"
  line="${line}${aux_line}\n"
  comment="\[init subscribers\]"
  add_line_to_file "\ \ ${line}" "${comment}" "${node_c}"

  if [[ "${msg_file}" = "Image" ]]
  then
    aux_line="void ${class_name}::${callback}(const ${msg_pkg}::${msg_file}::ConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info)"
    line="${aux_line}\n"
  else
    aux_line="void ${class_name}::${callback}(const ${msg_pkg}::${msg_file}::ConstPtr& msg)"
    line="${aux_line}\n"
  fi
  aux_line="{"
  line="${line}${aux_line}\n"
  aux_line="\ \ ROS_INFO(\"${class_name}::${callback}: New Message Received\");"
  line="${line}${aux_line}\n\n"
  aux_line="\ \ //use appropiate mutex to shared variables if necessary"
  line="${line}${aux_line}\n"
  aux_line="\ \ //this->${driver_alg}_.lock();"
  line="${line}${aux_line}\n"
  aux_line="\ \ //this->${mutex_name}enter();"
  line="${line}${aux_line}\n\n"
  aux_line="\ \ //std::cout << msg->data << std::endl;"
  line="${line}${aux_line}\n"
  if [[ "${msg_file}" = "Image" ]]
  then
    aux_line="\ \ // Uncomment the following line to convert the input image to OpenCV format"
    line="${line}${aux_line}\n"
    aux_line="\ \ //this->cv_image_ = cv_bridge::toCvCopy(msg, \"mono8\");"
    line="${line}${aux_line}\n\n"
  fi
  aux_line="\ \ //unlock previously blocked shared variables"
  line="${line}${aux_line}\n"
  aux_line="\ \ //this->${driver_alg}_.unlock();"
  line="${line}${aux_line}\n"
  aux_line="\ \ //this->${mutex_name}exit();"
  line="${line}${aux_line}\n"
  aux_line="}"
  line="${line}${aux_line}\n\n"
  aux_line="void ${class_name}::${mutex_name}enter(void)"
  line="${line}${aux_line}\n"
  aux_line="{"
  line="${line}${aux_line}\n"
  aux_line="\ \ pthread_mutex_lock(&this->${mutex_name});"
  line="${line}${aux_line}\n"
  aux_line="}"
  line="${line}${aux_line}\n\n"
  aux_line="void ${class_name}::${mutex_name}exit(void)"
  line="${line}${aux_line}\n"
  aux_line="{"
  line="${line}${aux_line}\n"
  aux_line="\ \ pthread_mutex_unlock(&this->${mutex_name});"
  line="${line}${aux_line}\n"
  aux_line="}"
  line="${line}${aux_line}\n"
  comment="\[subscriber callbacks\]"
  add_line_to_file "${line}" "${comment}" "${node_c}"

  line="pthread_mutex_destroy(&this->${mutex_name});"
  comment="\[free dynamic memory\]"
  add_line_to_file "\ \ ${line}" "${comment}" "${node_c}"

################################################################################
# Modify package.xml
# check if the message package is the current ros package

  add_build_run_dependencies "${ros_pkg}" "${msg_pkg}"
  if [[ "${msg_file}" = "Image" ]]
  then
    add_build_run_dependencies "${ros_pkg}" "camera_info_manager"
    add_build_run_dependencies "${ros_pkg}" "image_transport"
    add_build_run_dependencies "${ros_pkg}" "cv_bridge"
  fi

################################################################################
# modify the CMakeLists.txt file
# check if the message package is the current ros package

  add_cmake_dependencies "${ros_pkg}" "${msg_pkg}"
  if [[ "${msg_file}" = "Image" ]]
  then
    add_cmake_dependencies "${ros_pkg}" "camera_info_manager"
    add_cmake_dependencies "${ros_pkg}" "image_transport"
    add_cmake_dependencies "${ros_pkg}" "cv_bridge"
  fi

################################################################################
#compile
  goto_catkin_workspace
  catkin_make --only-pkg-with-deps ${ros_pkg}
}
