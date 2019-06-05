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

check_libraries
check_templates

echo ""
echo "/**********************************************/"
echo "/* Creating New ROS Node */"
echo "/**********************************************/"

usage="Usage: create_ros_node.sh -n node_name."
input_name=

#check for input project name paramenter
while getopts “:n:i” OPTION
do
  case $OPTION in
    n)
       input_name=$OPTARG
       ;;
    ?)
       echo "invalid argument $OPTION"
       kill_exit "${usage}"
       ;;
  esac
done

#check if project parameter is filled up
if [ ! "${input_name}" ]
then
  echo "No project name provided, aborting ..."
  kill_exit "${usage}"
fi

#lowercase input name
input_name=$(echo ${input_name} | tr "[:upper:]" "[:lower:]")

#create alg filename
project_name="${input_name}"

if [ -e "../${project_name}" ]
then
  kill_exit "${project_name} package directory already exists, aborting ..."
else
  echo "Generating folder structure for project ${project_name} ..."
fi

catkin_create_pkg ${project_name} roscpp roslib dynamic_reconfigure
if [ $? -eq 0 ]
then
  echo "Package created successfully"
  #remove automatically created folder, and no longer needed
  rm -rf ${project_name}/include/${project_name}
else
  exit 1;
fi

#create main class filename
class_filename="${input_name}"

#create alg filename
alg_filename="${input_name}_alg"

#create node filename
node_filename="${input_name}_node"

#create cfg filename
cfg_filename="${input_name}_config"

#create basename from pkg name
create_basename ${input_name}

#create templates folder path name
temps_folder="${SCRIPTS_PATH}/ros_node_templates/"


# Create folders
mkdir -p ${project_name}/include/
mkdir -p ${project_name}/src/

#create node basename
class_basename="${basename}Node"
#create alg basename
alg_basename="${basename}Algorithm"
#create node basename
node_basename="${basename}Node"

################################################################################
# Alg .h and .cpp files

#Set the filename and namespace on the template_alg files
sed -e "s/alg_filename/${alg_filename}/g" \
    -e "s/ClassAlg/${alg_basename}/g" \
    -e "s/project_name/${project_name}/g" \
    -e "s/BasenameConfig/${basename}Config/g" <${temps_folder}/template_alg.h >"${project_name}/include/${alg_filename}.h"
sed -e "s/alg_filename/${alg_filename}/g" \
    -e "s/project_name/${project_name}/g" \
    -e "s/ClassAlg/${alg_basename}/g" <${temps_folder}/template_alg.cpp >"${project_name}/src/${alg_filename}.cpp"
echo "Creating ${alg_filename} files..."
################################################################################

################################################################################
# Class .h and .cpp files

#Set the filename and namespace on the template_node files
sed -e "s/alg_filename/${alg_filename}/g" \
    -e "s/ClassAlg/${alg_basename}/g" \
    -e "s/class_filename/${class_filename}/g" \
    -e "s/project_name/${project_name}/g" \
    -e "s/Class/${class_basename}/g" <${temps_folder}/template.h >"${project_name}/include/${class_filename}.h"
sed -e "s/class_filename/${class_filename}/g" \
    -e "s/project_name/${project_name}/g" \
    -e "s/ClassNode/${class_basename}/g" <${temps_folder}/template.cpp >"${project_name}/src/${class_filename}.cpp"
echo "Creating ${node_filename} files..."
################################################################################

################################################################################
# Node .h and .cpp files

#Set the filename and namespace on the template_node files
sed -e "s/node_filename/${node_filename}/g" \
    -e "s/class_filename/${class_filename}/g" <${temps_folder}/template_node.h >"${project_name}/include/${node_filename}.h"
sed -e "s/project_name/${project_name}/g" \
    -e "s/node_filename/${node_filename}/g" \
    -e "s/ClassNode/${class_basename}/g" <${temps_folder}/template_node.cpp >"${project_name}/src/${node_filename}.cpp"
echo "Creating ${node_filename} files..."
################################################################################

################################################################################
# create launch files

mkdir -p ${project_name}/launch/

sed -e "s/project_name/${project_name}/g" <${temps_folder}/template_launch.launch >"${project_name}/launch/${node_filename}.launch"
echo "Creating ${node_filename} launch file..."
################################################################################


################################################################################
#Set the filename and namespace on the CMakeLists.txt file
sed -e "s/project_name/${project_name}/g" \
    -e "s/alg_filename/${alg_filename}/g" \
    -e "s/class_filename/${class_filename}/g" \
    -e "s/node_filename/${node_filename}/g" \
    -e "s/basename/${basename}/g" <${temps_folder}/CMakeLists.txt >"${project_name}/CMakeLists.txt"
echo "Creating ${project_name} CMakeLists.txt file..."
################################################################################


################################################################################
#create cfg directory
cfg_dir="${project_name}/cfg"
if [ -e "$cfg_dir" ]
then
  echo "${cfg_dir} directory already exists, skipping ..."
else
  echo "Creating ${cfg_dir} directory"
  mkdir ${cfg_dir}
fi

#Set the filename and namespace on the template.cfg file
sed -e "s/project_name/${project_name}/g" \
    -e "s/alg_basename/${alg_basename}/g" \
    -e "s/basename/${basename}/g" <${temps_folder}/template_alg.cfg >"${project_name}/cfg/${basename}.cfg"
eval "chmod 775 ${project_name}/cfg/${basename}.cfg"
echo "Creating ${cfg_filename}.cfg file..."
################################################################################

echo "Project ${project_name} has been successfully created!!"

pushd "${project_name}"
change_license_to_LGPL
popd

# WET
goto_catkin_workspace

# Uncomment this line depending on your needs
#catkin_make --only-pkg-with-deps ${project_name}
catkin build ${project_name}
