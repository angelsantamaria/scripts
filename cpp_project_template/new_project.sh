#! /bin/sh

DEP=
NAME=

while getopts “t:p:n:d:si” OPTION
do
  case $OPTION in
    n)
       NAME=$OPTARG
       ;;
    d)
       DEP=$OPTARG
       ;;
    h) 
       echo ******************************************  
       echo Script to generate a CPP project structure
       echo ******************************************
       echo Options:
       echo   n: Project name
       echo   d: Dependency names to be added to the project
       echo   
       echo Example of usage:
       echo ./new_project.sh -n example_project 
    ?)
       echo invalid argument $OPTION
       exit
       ;;
  esac
done

echo Generating project ...

if [ $NAME ]
then
  NAME=$(echo $NAME | tr '[:upper:]' '[:lower:]')
  echo "Using library name $NAME ..."
else
  echo "No library name provided, aborting ..."
  exit
fi

#create the bin directory
BIN_DIR="./bin"
if [ -e "$BIN_DIR" ]
then
  echo "$BIN_DIR directory already exists, skipping ..."
else
  echo "Creating $BIN_DIR directory"
  mkdir $BIN_DIR
fi  

# create the lib directory
LIB_DIR="./lib"
if [ -e "$LIB_DIR" ]
then
  echo "$LIB_DIR directory already exists, skipping ..."
else
  echo "Creating $LIB_DIR directory"
  mkdir $LIB_DIR
fi  

# create the src directory
SRC_DIR="./src"
if [ -e "$SRC_DIR" ]
then
  echo "$SRC_DIR directory already exists, skipping ..."
else
  echo "Creating $SRC_DIR directory"
  mkdir $SRC_DIR
fi  

#create the src/examples directory
EXAMPLES_DIR="./src/examples"
if [ -e "$EXAMPLES_DIR" ]
then
  echo "$EXAMPLES_DIR directory already exists, skipping ..."
else
  echo "Creating $EXAMPLES_DIR directory"
  mkdir $EXAMPLES_DIR
fi  
fi

# create the build directory
BUILD_DIR="./build"
if [ -e "$BUILD_DIR" ]
then
  echo "$BUILD_DIR directory already exists, skipping ..."
else
  echo "Creating $BUILD_DIR directory"
  mkdir $BUILD_DIR
fi  

#create the doc directory
DOC_DIR="./doc"
if [ -e "$DOC_DIR" ]
then
  echo "$DOC_DIR directory already exists, skipping ..."
else
  echo "Creating $DOC_DIR directory"
  mkdir $DOC_DIR
fi  

#create the doc/images directory
IMAGES_DIR="./doc/images"
if [ -e "$IMAGES_DIR" ]
then
  echo "$IMAGES_DIR directory already exists, skipping ..."
else
  echo "Creating $IMAGES_DIR directory"
  mkdir $IMAGES_DIR
fi  

# create the doc/files directory
FILES_DIR="./doc/files"
if [ -e "$FILES_DIR" ]
then
  echo "$FILES_DIR directory already exists, skipping ..."
else
  echo "Creating $FILES_DIR directory"
  mkdir $FILES_DIR
fi  

#parse the dependencies and create the dependencies file
arr=$(echo $DEP | tr "," "\n")

#Set the project name on the ReadMe.txt disclaimer file
sed 's/project_name/'$NAME'/g' <ReadMe_template.md >./ReadMe.md
rm ReadMe_template.md

#Set the project name on the CMakeLists.txt script file
sed 's/project_name/'$NAME'/g' <CMakeLists_template.txt >./CMakeLists.txt
rm CMakeLists_template.txt

#Set the project name on the doxygen_project_name.dox sript file
sed 's/project_name/'"$NAME"'/g' <doxygen_project_name_template.conf >./doc/doxygen_project_name.conf
rm doxygen_project_name_template.conf
mv doxygen_template.conf ./doc/doxygen.conf

#create the CMakeLists.txt script file
echo "# library source files" >> CMakeLists.tmp
echo "SET(sources ${NAME}.cpp)" >> CMakeLists.tmp
echo "" >> CMakeLists.tmp
echo "# application header files" >> CMakeLists.tmp
echo "SET(headers ${NAME}.h)" >> CMakeLists.tmp
echo "" >> CMakeLists.tmp
echo "# locate the necessary dependencies" >> CMakeLists.tmp
for x in $arr
do
  echo "FIND_PACKAGE($x REQUIRED)" >> CMakeLists.tmp
done
echo "" >> CMakeLists.tmp
echo "# add the necessary include directories" >> CMakeLists.tmp
echo "INCLUDE_DIRECTORIES(.)" >> CMakeLists.tmp
for x in $arr
do
  echo "INCLUDE_DIRECTORIES("'${'"${x}_INCLUDE_DIR"'}'")" >> CMakeLists.tmp
done
echo "" >> CMakeLists.tmp
echo "# create the shared library" >> CMakeLists.tmp
echo "ADD_LIBRARY(${NAME} SHARED "'${'"sources"'}'")" >> CMakeLists.tmp
echo "" >> CMakeLists.tmp
echo "# link necessary libraries" >> CMakeLists.tmp
for x in $arr
do
  echo "TARGET_LINK_LIBRARIES(${NAME} "'${'"${x}_LIBRARY"'}'")" >> CMakeLists.tmp
done
echo "" >> CMakeLists.tmp
echo "INSTALL(TARGETS $NAME" >> CMakeLists.tmp
echo "        RUNTIME DESTINATION bin" >> CMakeLists.tmp
echo "        LIBRARY DESTINATION lib/${LIBRARY_NAME}" >> CMakeLists.tmp
echo "        ARCHIVE DESTINATION lib/${LIBRARY_NAME})" >> CMakeLists.tmp
echo "INSTALL(FILES "'${'"headers"'}' "DESTINATION include/${LIBRARY_NAME})" >> CMakeLists.tmp
echo "INSTALL(FILES ../Find$NAME.cmake DESTINATION "'${'"CMAKE_ROOT"'}'"/Modules/)" >> CMakeLists.tmp
echo "" >> CMakeLists.tmp
echo "ADD_SUBDIRECTORY(examples)" >> CMakeLists.tmp
mv CMakeLists.tmp ./src/CMakeLists.txt
  
echo "# create an example application" >> CMakeLists.tmp
echo "ADD_EXECUTABLE(${NAME}_test ${NAME}_test.cpp)" >> CMakeLists.tmp
echo "" >> CMakeLists.tmp
echo "# link necessary libraries" >> CMakeLists.tmp
echo "TARGET_LINK_LIBRARIES(${NAME}_test $NAME)" >> CMakeLists.tmp
for x in $arr
do
  echo "TARGET_LINK_LIBRARIES(${NAME}_test "'${'"${x}_LIBRARY"'}'")" >> CMakeLists.tmp
done
mv CMakeLists.tmp ./src/examples/CMakeLists.txt

sed 's/header_file/'"${NAME}.h"'/g' <Findlib_template.cmake >tmp.cmake
sed 's/library_name/'$NAME'/g' <tmp.cmake >./Find$NAME.cmake
rm tmp.cmake

LIBRARY_NAME=$(echo $NAME | tr '[:lower:]' '[:upper:]')
Library_name=$(echo $NAME | sed 's/\([a-zA-Z]\)\([a-zA-Z0-9]*\)/\u\1\2/g')
sed 's/Library_name/'$Library_name'/g' <library_header_template.h >tmp.h
sed 's/LIBRARY_NAME/'$LIBRARY_NAME'/g' <tmp.h >./src/$NAME.h
rm tmp.h
  
sed 's/library_name/'$NAME'/g' <library_src_template.cpp >tmp.cpp
sed 's/Library_name/'$Library_name'/g' <tmp.cpp >tmp2.cpp
sed 's/Library_name/'$Library_name'/g' <tmp2.cpp >./src/$NAME.cpp
rm tmp.cpp
rm tmp2.cpp

sed 's/library_name/'$NAME'/g' <example_src_template.cpp >./src/examples/${NAME}_test.cpp

sed 's/library_name/'$NAME'/g' <main_template.dox >tmp.dox
sed 's/project_name/'"$NAME"'/g' <tmp.dox >./doc/main.dox
rm library_src_template.cpp
rm library_header_template.h
rm library_example_src_template.cpp
rm Findlib_template.cmake
rm main_template.dox
rm tmp.dox
rm template.tar.gz
rm new_project.sh
