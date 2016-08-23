cmake_minimum_required(VERSION 2.8)

# add prefix to every string in a variable
FUNCTION(PREPEND var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND)

FUNCTION(PREPEND_CPP var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}.cpp")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND_CPP)

FUNCTION(PREPEND_H var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}.h")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND_H)

project(Kinect2Calibration)

find_package(PCL 1.8 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

if(UNIX)
        set(CMAKE_C_COMPILER "/usr/bin/gcc")
        set(CMAKE_CXX_COMPILER "/usr/bin/g++")
#	set(CMAKE_C_COMPILER "/usr/bin/clang")
#	set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
endif(UNIX)

# set binary and library folders
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

# set debug and release options
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CONFIGURATION_TYPES Debug Release)

# set compile options
if(UNIX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(UNIX)

# set definitions to disable some warning
if(WIN32)
add_definitions(-DWIN32 -D_CRT_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_WARNINGS)
endif(WIN32)
if(UNIX)
add_definitions(-DUNIX)
endif(UNIX)

# set include and lib folders
if(UNIX)
set(GENERAL_INCL_DIR "/usr/Aris_Dependent")
set(SYSTEM_LINK_LIB pthread)
set(XENOMAI_INCL_DIR "/usr/xenomai/include")
set(XENOMAI_LINK_DIR "/usr/xenomai/lib")
set(XENOMAI_LINK_LIB native rtdm xenomai)
set(ETHERLAB_INCL_DIR "/opt/etherlab/include")
set(ETHERLAB_LINK_DIR "/opt/etherlab/lib")
set(ETHERLAB_LINK_LIB ethercat rt)
set(XSENS_INCL_DIR "/usr/Aris_Dependent/xsens/include")
set(XSENS_LINK_LIB xdeviceclass xcommunication xstypes)
set(XSENS_LINK_DIR "/usr/Aris_Dependent/xsens/lib")
set(PCL_INCL_DIR ${PCL_INCLUDE_DIRS})#pcl for velodyne
set(PCL_LINK_DIR ${PCL_LIBRARY_DIRS})
set(PCL_LINK_LIB ${PCL_LIBRARIES})
set(ARIS_INCL_DIR "/usr/aris/include")
set(ARIS_LINK_DIR "/usr/aris/lib")
set(ARIS_LINK_LIB aris_server aris_sensor aris_dynamic aris_control aris_core)
set(ROBOTS_INCL_DIR "/usr/Robots/include")
set(ROBOTS_LINK_DIR "/usr/Robots/lib")
set(ROBOTS_LINK_LIB Robot_Type_I Robot_Type_II Robot_Base)

set(VISI_INCL_DIR "/usr/local/include/libfreenect2" "./Kinect2")
set(VISI_LINK_DIR "/usr/local/lib")
set(VISI_LINK_LIB freenect2)

set(ALL_INCL_DIR ${VISI_INCL_DIR} ${ROBOTS_INCL_DIR} ${ARIS_INCL_DIR} ${PCL_INCL_DIR} ${XSENS_INCL_DIR} ${ETHERLAB_INCL_DIR} ${XENOMAI_INCL_DIR} ${GENERAL_INCL_DIR})
set(ALL_LINK_DIR ${VISI_LINK_DIR} ${ROBOTS_LINK_DIR} ${ARIS_LINK_DIR} ${PCL_LINK_DIR} ${XSENS_LINK_DIR} ${ETHERLAB_LINK_DIR} ${XENOMAI_LINK_DIR})
set(ALL_LINK_LIB ${VISI_LINK_LIB} ${ROBOTS_LINK_LIB} ${ARIS_LINK_LIB} ${PCL_LINK_LIB} ${XSENS_LINK_LIB} ${ETHERLAB_LINK_LIB} ${XENOMAI_LINK_LIB} ${SYSTEM_LINK_LIB})
endif(UNIX)
if(WIN32)
set(GENERAL_INCL_DIR "C:/Aris_Dependent")
set(SYSTEM_LINK_LIB ws2_32 setupapi)
set(XSENS_INCL_DIR "C:/Aris_Dependent/xsens/include")
set(XSENS_LINK_DIR "C:/Aris_Dependent/xsens/lib")
set(XSENS_LINK_LIB xdeviceclass)
set(ARIS_INCL_DIR "C:/Aris/include")
set(ARIS_LINK_DIR "C:/Aris/lib")
set(ARIS_LINK_LIB aris_server aris_sensor aris_dynamic aris_control aris_core)
set(ROBOTS_INCL_DIR "C:/Robots/include")
set(ROBOTS_LINK_DIR "C:/Robots/lib")
set(ROBOTS_LINK_LIB Robot_Type_I Robot_Type_II Robot_Base)

set(ALL_INCL_DIR ${ROBOTS_INCL_DIR} ${ARIS_INCL_DIR} ${XSENS_INCL_DIR} ${ETHERLAB_INCL_DIR} ${XENOMAI_INCL_DIR} ${GENERAL_INCL_DIR})
set(ALL_LINK_DIR ${ROBOTS_LINK_DIR} ${ARIS_LINK_DIR} ${XSENS_LINK_DIR} )
set(ALL_LINK_LIB ${ROBOTS_LINK_LIB} ${ARIS_LINK_LIB} ${XSENS_LINK_LIB} ${SYSTEM_LINK_LIB})
endif(WIN32)

include_directories(${ALL_INCL_DIR})
link_directories(${ALL_LINK_DIR})

################################### build server ####################################
set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC Server ${SOURCE_FILES})

set(VISION_SOURCE_FILES Kinect2.cpp Kinect2Test.cpp Vision_Gait0.cpp)
PREPEND(VISION_FULL_SRC Kinect2 ${VISION_SOURCE_FILES})

add_executable(Server ${VISION_FULL_SRC} ${FULL_SRC})
target_link_libraries(Server ${ALL_LINK_LIB})

add_executable(rbt Client/client.cpp)
target_link_libraries(rbt ${ALL_LINK_LIB})





































