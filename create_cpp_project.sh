#! /bin/sh

# check wether the scripts path environment variable has been defined
SCRIPTS_PATH=`echo "${SCRIPTS_PATH}"`
if [ -z "${SCRIPTS_PATH}" ]
then
  echo "The scripts path environment varibale has not been defined. Please see the wiki documentation for instructions on how to create it."
  exit
fi

TEMPLATES_PATH=$SCRIPTS_PATH/cpp_project_template

DEP=
NAME=

while getopts "h:n:d" OPTION
do
  case $OPTION in
    h) 
       echo "******************************************"  
       echo "Script to generate a CPP project structure"
       echo "******************************************"
       echo "Options:"
       echo "  -n: Project name"
       echo "  -d: Dependency names to be added to the project"
       echo ""  
       echo "Example of usage:"
       echo "new_project.sh -n example_project" 
       echo ""
       exit
       ;;
    n)
       NAME=$OPTARG
       ;;
    d)
       DEP=$OPTARG
       ;;
    ?)
       echo "type -h for help"
       exit
       ;;
  esac
done

#Original name only for folder name
ORIGNAME=$NAME

#Lower case name
if [ $NAME ]
then
  NAME=$(echo $NAME | tr '[:upper:]' '[:lower:]')
else
  echo "No library name provided, aborting ..."
  exit
fi

echo Generating $ORIGNAME project

#create the project directory
if [ -e "$ORIGNAME" ]
then
  echo "  ! $ORIGNAME directory already exists, skipping ..."
else
  echo "  > Creating $ORIGNAME directory"
  mkdir $ORIGNAME
fi  

#create the bin directory
BIN_DIR="$ORIGNAME/bin"
if [ -e "$BIN_DIR" ]
then
  echo "  ! $BIN_DIR directory already exists, skipping ..."
else
  echo "  > Creating $BIN_DIR directory"
  mkdir $BIN_DIR
fi  

# create the lib directory
LIB_DIR="$ORIGNAME/lib"
if [ -e "$LIB_DIR" ]
then
  echo "  ! $LIB_DIR directory already exists, skipping ..."
else
  echo "  > Creating $LIB_DIR directory"
  mkdir $LIB_DIR
fi  

# create the src directory
SRC_DIR="$ORIGNAME/src"
if [ -e "$SRC_DIR" ]
then
  echo "  ! $SRC_DIR directory already exists, skipping ..."
else
  echo "  > Creating $SRC_DIR directory"
  mkdir $SRC_DIR
fi  

#create the src/examples directory
EXAMPLES_DIR="$ORIGNAME/src/examples"
if [ -e "$EXAMPLES_DIR" ]
then
  echo "  ! $EXAMPLES_DIR directory already exists, skipping ..."
else
  echo "  > Creating $EXAMPLES_DIR directory"
  mkdir $EXAMPLES_DIR
fi  

# create the build directory
BUILD_DIR="$ORIGNAME/build"
if [ -e "$BUILD_DIR" ]
then
  echo "  ! $BUILD_DIR directory already exists, skipping ..."
else
  echo "  > Creating $BUILD_DIR directory"
  mkdir $BUILD_DIR
fi  

#create the doc directory
DOC_DIR="$ORIGNAME/doc"
if [ -e "$DOC_DIR" ]
then
  echo "  ! $DOC_DIR directory already exists, skipping ..."
else
  echo "  > Creating $DOC_DIR directory"
  mkdir $DOC_DIR
fi  

#create the doc/images directory
IMAGES_DIR="$ORIGNAME/doc/images"
if [ -e "$IMAGES_DIR" ]
then
  echo "  ! $IMAGES_DIR directory already exists, skipping ..."
else
  echo "  > Creating $IMAGES_DIR directory"
  mkdir $IMAGES_DIR
fi  

# create the doc/files directory
FILES_DIR="$ORIGNAME/doc/files"
if [ -e "$FILES_DIR" ]
then
  echo "  ! $FILES_DIR directory already exists, skipping ..."
else
  echo "  > Creating $FILES_DIR directory"
  mkdir $FILES_DIR
fi  

#parse the dependencies and create the dependencies file
arr=$(echo $DEP | tr "," "\n")

#Set the project name on the ReadMe.txt disclaimer file
sed 's/project_name/'$ORIGNAME'/g' <$TEMPLATES_PATH/ReadMe_template.md >tmp.md
sed 's/library_name/'$NAME'/g' <tmp.md >$ORIGNAME/ReadMe.md
rm tmp.md

#Set the project name on the CMakeLists.txt script file
sed 's/project_name/'$NAME'/g' <$TEMPLATES_PATH/CMakeLists_template.txt >$ORIGNAME/CMakeLists.txt

#Set the project name on the doxygen_project_name.dox sript file
sed 's/project_name/'"$ORIGNAME"'/g' <$TEMPLATES_PATH/doxygen_project_name_template.conf >$ORIGNAME/doc/doxygen_project_name.conf
cp $TEMPLATES_PATH/doxygen_template.conf $ORIGNAME/doc/doxygen.conf

#create the CMakeLists.txt script file
echo "# library source files" >> CMakeLists.tmp
echo "SET(sources ${NAME}.cpp)" >> CMakeLists.tmp
echo "# application header files" >> CMakeLists.tmp
echo "SET(headers ${NAME}.h)" >> CMakeLists.tmp
echo "# locate the necessary dependencies" >> CMakeLists.tmp
for x in $arr
do
  echo "FIND_PACKAGE($x REQUIRED)" >> CMakeLists.tmp
done
echo "# add the necessary include directories" >> CMakeLists.tmp
echo "INCLUDE_DIRECTORIES(.)" >> CMakeLists.tmp
for x in $arr
do
  echo "INCLUDE_DIRECTORIES("'${'"${x}_INCLUDE_DIR"'}'")" >> CMakeLists.tmp
done
echo "# create the shared library" >> CMakeLists.tmp
echo "ADD_LIBRARY(${NAME} SHARED "'${'"sources"'}'")" >> CMakeLists.tmp
echo "# link necessary libraries" >> CMakeLists.tmp
for x in $arr
do
  echo "TARGET_LINK_LIBRARIES(${NAME} "'${'"${x}_LIBRARY"'}'")" >> CMakeLists.tmp
done
echo "INSTALL(TARGETS $NAME" >> CMakeLists.tmp
echo "        RUNTIME DESTINATION bin" >> CMakeLists.tmp
echo "        LIBRARY DESTINATION lib/${NAME}" >> CMakeLists.tmp
echo "        ARCHIVE DESTINATION lib/${NAME})" >> CMakeLists.tmp
echo "INSTALL(FILES "'${'"headers"'}' "DESTINATION include/${NAME})" >> CMakeLists.tmp
echo "INSTALL(FILES ../Find$NAME.cmake DESTINATION "'${'"CMAKE_ROOT"'}'"/Modules/)" >> CMakeLists.tmp
echo "ADD_SUBDIRECTORY(examples)" >> CMakeLists.tmp
mv CMakeLists.tmp $ORIGNAME/src/CMakeLists.txt
  
echo "# create an example application" >> CMakeLists.tmp
echo "ADD_EXECUTABLE(${NAME}_test ${NAME}_test.cpp)" >> CMakeLists.tmp
echo "# link necessary libraries" >> CMakeLists.tmp
echo "TARGET_LINK_LIBRARIES(${NAME}_test $NAME)" >> CMakeLists.tmp
for x in $arr
do
  echo "TARGET_LINK_LIBRARIES(${NAME}_test "'${'"${x}_LIBRARY"'}'")" >> CMakeLists.tmp
done
mv CMakeLists.tmp $ORIGNAME/src/examples/CMakeLists.txt

sed 's/header_file/'"${NAME}.h"'/g' <$TEMPLATES_PATH/Findlib_template.cmake >tmp.cmake
sed 's/library_name/'$NAME'/g' <tmp.cmake >$ORIGNAME/Find$NAME.cmake
rm tmp.cmake

LIBRARY_NAME=$(echo $NAME | tr '[:lower:]' '[:upper:]')
Library_name=$(echo $NAME | sed 's/\([a-zA-Z]\)\([a-zA-Z0-9]*\)/\u\1\2/g')
sed 's/Library_name/'$Library_name'/g' <$TEMPLATES_PATH/library_header_template.h >tmp.h
sed 's/LIBRARY_NAME/'$LIBRARY_NAME'/g' <tmp.h >$ORIGNAME/src/$NAME.h
rm tmp.h
  
sed 's/library_name/'$NAME'/g' <$TEMPLATES_PATH/library_src_template.cpp >tmp.cpp
sed 's/Library_name/'$Library_name'/g' <tmp.cpp >tmp2.cpp
sed 's/Library_name/'$Library_name'/g' <tmp2.cpp >$ORIGNAME/src/$NAME.cpp
rm tmp.cpp
rm tmp2.cpp

sed 's/library_name/'$NAME'/g' <$TEMPLATES_PATH/example_src_template.cpp >$ORIGNAME/src/examples/${NAME}_test.cpp

sed 's/library_name/'$NAME'/g' <$TEMPLATES_PATH/main_template.dox >tmp.dox
sed 's/project_name/'"$ORIGNAME"'/g' <tmp.dox >$ORIGNAME/doc/main.dox
rm tmp.dox

#create gitignores
echo "  > Creating gitignore files"
if [ -f $BIN_DIR/.gitignore ]
then
  echo "    ! gitignore file already exist in bin directory, skipping ..."
else
  echo "    > creating .gitignore file in bin directory"	
  cp $SCRIPTS_PATH/cpp_project_template/gitignore_template $BIN_DIR/.gitignore
fi

if [ -f $BUILD_DIR/.gitignore ]
then
  echo "    ! gitignore file already exist in build directory, skipping ..."
else
  echo "    > creating .gitignore file in build directory"	
  cp $SCRIPTS_PATH/cpp_project_template/gitignore_template $BUILD_DIR/.gitignore
fi

if [ -f $LIB_DIR/.gitignore ]
then
  echo "    ! gitignore file already exist in lib directory, skipping ..."
else
  echo "    > creating .gitignore file in lib directory"	
  cp $SCRIPTS_PATH/cpp_project_template/gitignore_template $LIB_DIR/.gitignore
fi

echo "Project created."

