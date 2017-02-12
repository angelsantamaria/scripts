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

function check_publisher_file_integrity
{
  local node_h=$1
  local node_c=$2
  local comment=""

  # check the node.h file
  comment="\[publisher subscriber headers\]"
  find_comment_in_file "${comment}" "${node_h}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[publisher subscriber headers\] from the header file ${node_h}"
  fi
  comment="\[publisher attributes\]"
  find_comment_in_file "${comment}" "${node_h}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[publisher attributes\] from the header file ${node_h}"
  fi

  # check the node.cpp file
  comment="\[init publishers\]"
  find_comment_in_file "${comment}" "${node_c}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[init publishers\] from the header file ${node_c}"
  fi
  comment="\[fill msg structures\]"
  find_comment_in_file "${comment}" "${node_c}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[fill msg structures\] from the header file ${node_c}"
  fi

  comment="\[publish messages\]"
  find_comment_in_file "${comment}" "${node_c}"
  if [[ "${comment_found}" = "false" ]]
  then
    kill_exit "Missing \[publish messages\] from the header file ${node_c}"
  fi
}

function check_publisher_attributes_functions
{
  local node_h=$1
  local node_c=$2
  local msg_pkg=$3
  local msg_file=$4
  local topic_name=$5
  local publisher_name=$6
  local buffer=$7
  local line=""

  # check the node.h file
  if [[ "${msg_file}" = "Image" ]]
  then
    line="image_transport::CameraPublisher ${publisher_name};"
    find_comment_in_file "${line}" "${node_h}"
    if [[ "${comment_found}" = "true" ]]
    then
      kill_exit "A publisher with the same name is already declared in file ${node_h} line ${line_number}"
    fi
    line="camera_info_manager::CameraInfoManager ${topic_name}_camera_manager;"
    find_comment_in_file "${line}" "${node_h}"
    if [[ "${comment_found}" = "true" ]]
    then
      kill_exit "A camera manager with the same name is already declared in file ${node_h} line ${line_number}"
    fi
  else
    line="ros::Publisher ${publisher_name};"
    find_comment_in_file "${line}" "${node_h}"
    if [[ "${comment_found}" = "true" ]]
    then
      kill_exit "A publisher with the same name is already declared in file ${node_h} line ${line_number}"
    fi
  fi

  line="${msg_pkg}::${msg_file} ${topic_name}_msg_;"
  find_comment_in_file "${line}" "${node_h}"
  if [[ "${comment_found}" = "true" ]]
  then
    kill_exit "A message variable with the same name is already declared in file ${node_h} line ${line_number}"
  fi

  # check the node.cpp file
  if [[ "${msg_file}" = "Image" ]]
  then
    line="${topic_name}_camera_manager(ros::NodeHandle(\"~${topic_name}\"))"
    find_comment_in_file "${line}" "${node_c}"
    if [[ "${comment_found}" = "true" ]]
    then
      kill_exit "A camera manager with the same name is already initialized in file ${node_c} line ${line_number}"
    fi
    line="this->${publisher_name} = this->it.advertiseCamera(\"${topic_name}/image_raw\", ${buffer});"
    find_comment_in_file "${line}" "${node_c}"
    if [[ "${comment_found}" = "true" ]]
    then
      kill_exit "A topic with the same name is already advertised in file ${node_c} line ${line_number}"
    fi
    line="this->${topic_name}_camera_manager.validateURL(${topic_name}_cal_file)"
    find_comment_in_file "${line}" "${node_c}"
    if [[ "${comment_found}" = "true" ]]
    then
      kill_exit "The calibration file URL for a camera manager with the same name is already validated in file ${node_c} line ${line_number}"
    fi
    line="this->${topic_name}_camera_manager.loadCameraInfo(${topic_name}_cal_file)"
    find_comment_in_file "${line}" "${node_c}"
    if [[ "${comment_found}" = "true" ]]
    then
      kill_exit "The camera information for a camera manager with the same name is already loaded in file ${node_c} line ${line_number}"
    fi
    line="sensor_msgs::CameraInfo ${topic_name}_camera_info=this->${topic_name}_camera_manager.getCameraInfo();"
    find_comment_in_file "${line}" "${node_c}"
    if [[ "${comment_found}" = "true" ]]
    then
      kill_exit "The camera info structure for a topic with the same name is already initialized in file ${node_c} line ${line_number}"
    fi
    line="//this->${publisher_name}.publish(this->${topic_name}_msg_,${topic_name}_camera_info);"
    find_comment_in_file "${line}" "${node_c}"
    if [[ "${comment_found}" = "true" ]]
    then
      kill_exit "A message with the same name is already published in file ${node_c} line ${line_number}"
    fi
  else
    line="this->${publisher_name} = nh.advertise<${msg_pkg}::${msg_file}>(\"${topic_name}\", ${buffer});"
    find_comment_in_file "${line}" "${node_c}"
    if [[ "${comment_found}" = "true" ]]
    then
      kill_exit "A publisher with the same name is already advertised in file ${node_c} line ${line_number}"
    fi
    line="this->${publisher_name}.publish(this->${topic_name}_msg_);"
    find_comment_in_file "${line}" "${node_c}"
    if [[ "${comment_found}" = "true" ]]
    then
      kill_exit "A message with the same name is already published in file ${node_c} line ${line_number}"
    fi
  fi
}

function create_publisher
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
  local publisher_name="${topic_name}_publisher_"
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

  #check files integrity before making any changes
  check_cmakelists_file_integrity "${driver_alg}"
  check_publisher_file_integrity "${node_h}" "${node_c}"
  check_publisher_attributes_functions "${node_h}" "${node_c}" "${msg_pkg}" "${msg_file}" "${topic_name}" "${publisher_name}" "${buffer}"

################################################################################
# modify Node.h #

  #look for include files and add them if are not already there
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

  # add the message type variable
  aux_line="${msg_pkg}::${msg_file} ${topic_name}_msg_;"
  line="\ \ \ \ ${aux_line}\n"
  comment="\[publisher attributes\]"
  add_line_to_file "${line}" "${comment}" "${node_h}"

  # special actions for images
  if [[ "${msg_file}" = "Image" ]]
  then
    aux_line="image_transport::CameraPublisher ${publisher_name};"
    line="\ \ \ \ ${aux_line}"
    comment="\[publisher attributes\]"
    add_line_to_file "${line}" "${comment}" "${node_h}"
    aux_line="camera_info_manager::CameraInfoManager ${topic_name}_camera_manager;"
    line="\ \ \ \ ${aux_line}"
    comment="\[publisher attributes\]"
    add_line_to_file "${line}" "${comment}" "${node_h}"
    aux_line="image_transport::ImageTransport it;"
    find_comment_in_file "${aux_line}" "${node_h}"
    if [[ "${comment_found}" = "false" ]]
    then
      line="\ \ \ \ ${aux_line}"
      comment="\[publisher attributes\]"
      add_line_to_file "${line}\n" "${comment}" "${node_h}"
    fi
    aux_line="cv_bridge::CvImagePtr cv_image_;"
    find_comment_in_file "${aux_line}" "${node_h}"
    if [[ "${comment_found}" = "false" ]]
    then
      line="// Uncomment to use the openCV <-> ROS bridge"
      line="\ \ \ \ ${line}\n\ \ \ \ //${aux_line}"
      comment="\[publisher attributes\]"
      add_line_to_file "${line}" "${comment}" "${node_h}"
    fi
  else
    aux_line="ros::Publisher ${publisher_name};"
    line="\ \ \ \ ${aux_line}"
    comment="\[publisher attributes\]"
    add_line_to_file "${line}" "${comment}" "${node_h}"
  fi

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
    line="${topic_name}_camera_manager(ros::NodeHandle(\"~${topic_name}\"))"

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
    # try load the calibration file
    aux_line="// uncomment the following lines to load the calibration file for the camera"
    line="${aux_line}\n"
    aux_line="\ \ // Change <cal_file_param> for the correct parameter name holding the configuration filename"
    line="${line}${aux_line}\n"
    aux_line="\ \ //std::string ${topic_name}_cal_file;"
    line="${line}${aux_line}\n"
    aux_line="\ \ //nh.param<std::string>(\"<cal_file_param>\",${topic_name}_cal_file,\"\");"
    line="${line}${aux_line}\n"
    aux_line="\ \ //if(this->${topic_name}_camera_manager.validateURL(${topic_name}_cal_file))"
    line="${line}${aux_line}\n"
    aux_line="\ \ //{"
    line="${line}${aux_line}\n"
    aux_line="\ \ //\ \ if(!this->${topic_name}_camera_manager.loadCameraInfo(${topic_name}_cal_file))"
    line="${line}${aux_line}\n"
    aux_line="\ \ //\ \ \ \ ROS_INFO(\"Invalid calibration file\");"
    line="${line}${aux_line}\n"
    aux_line="\ \ //}"
    line="${line}${aux_line}\n"
    aux_line="\ \ //else"
    line="${line}${aux_line}\n"
    aux_line="\ \ //\ \ ROS_INFO(\"Invalid calibration file\");"
    line="${line}${aux_line}\n"
    comment="\[init publishers\]"
    add_line_to_file "  ${line}" "${comment}" "${node_c}"
    line="this->${publisher_name} = this->it.advertiseCamera(\"${topic_name}/image_raw\", ${buffer});"
    comment="\[init publishers\]"
    add_line_to_file "  ${line}" "${comment}" "${node_c}"
    aux_line="// Uncomment the following lines two initialize the camera info structure"
    line="${aux_line}\n"
    aux_line="\ \ //sensor_msgs::CameraInfo ${topic_name}_camera_info=this->${topic_name}_camera_manager.getCameraInfo();"
    line="${line}${aux_line}\n"
    aux_line="\ \ //${topic_name}_camera_info.header.stamp = <time_stamp>;"
    line="${line}${aux_line}\n"
    aux_line="\ \ //${topic_name}_camera_info.header.frame_id = <frame_id>;"
    line="${line}${aux_line}\n"
    comment="\[fill msg structures\]"
    add_line_to_file "\ \ ${line}" "${comment}" "${node_c}"
    aux_line="// Uncomment the following line to convert an OpenCV image to a ROS image message"
    line="${aux_line}\n"
    aux_line="\ \ //this->${topic_name}_msg_=*this->cv_image_->toImageMsg();"
    line="${line}${aux_line}\n"
    aux_line="\ \ // Uncomment the following line to publish the image together with the camera information"
    line="${line}${aux_line}\n"
    aux_line="\ \ //this->${publisher_name}.publish(this->${topic_name}_msg_,${topic_name}_camera_info);"
    line="${line}${aux_line}\n"
    comment="\[publish messages\]"
    add_line_to_file "\ \ ${line}" "${comment}" "${node_c}"
  else
    line="this->${publisher_name} = nh.advertise<${msg_pkg}::${msg_file}>(\"${topic_name}\", ${buffer});"
    comment="\[init publishers\]"
    add_line_to_file "  ${line}" "${comment}" "${node_c}"
    aux_line="// Uncomment the following line to publish the topic message"
    line="${aux_line}\n"
    aux_line="\ \ //this->${publisher_name}.publish(this->${topic_name}_msg_);"
    line="${line}${aux_line}\n"
    comment="\[publish messages\]"
    add_line_to_file "\ \ ${line}" "${comment}" "${node_c}"
  fi

  aux_line="// Initialize the topic message structure"
  line="${aux_line}\n"
  aux_line="\ \ //this->${topic_name}_msg_.data = my_var;"
  line="${line}${aux_line}\n"
  comment="\[fill msg structures\]"
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
