cmake_minimum_required(VERSION 3.10)

project(rknn_yolov8_demo)

if (ENABLE_ASAN)
	message(STATUS "BUILD WITH ADDRESS SANITIZER")
	set (CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
	set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
	set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
endif ()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/ 3rdparty.out)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../utils/ utils.out)
# zlmediakit
set(ZLMEDIAKIT_PATH ${CMAKE_SOURCE_DIR}/../../../3rdparty/zlmediakit)
#include_directories(${ZLMEDIAKIT_PATH}/include)
#set(ZLMEDIAKIT_LIBS ${ZLMEDIAKIT_PATH}/${LIB_ARCH}/libmk_api.so ${ZLMEDIAKIT_PATH}/${LIB_ARCH}/librtsp.a)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  include_directories(${ZLMEDIAKIT_PATH}/include)
  set(ZLMEDIAKIT_LIBS ${ZLMEDIAKIT_PATH}/${LIB_ARCH}/libmk_api.so ${ZLMEDIAKIT_PATH}/${LIB_ARCH}/librtsp.a)
endif()

if(ZLMEDIAKIT_LIBS)
  add_definitions(-DBUILD_VIDEO_RTSP)
endif()




# mpp
set(MPP_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/mpp)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(MPP_LIBS ${MPP_PATH}/Linux/aarch64/librockchip_mpp.so)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Android")
  set(MPP_LIBS ${MPP_PATH}/${CMAKE_SYSTEM_NAME}/${CMAKE_ANDROID_ARCH_ABI}/libmpp.so)
endif()

include_directories(${MPP_PATH}/include)


# rga


#qt5
find_package(Qt5 COMPONENTS Core REQUIRED)

#eigen
find_package(Eigen3 REQUIRED)

#ffmpeg
# 设置FFmpeg的头文件目录
set(FFMPEG_INCLUDE_DIRS /usr/include/aarch64-linux-gnu)

# 设置FFmpeg的库文件目录
set(FFMPEG_LIBRARIES /usr/lib/aarch64-linux-gnu)
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
find_package(OpenCV REQUIRED)
find_package(PkgConfig)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
file(GLOB SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)
find_package(SDL2 REQUIRED)
#find_package(FFmpeg REQUIRED)

add_executable(${PROJECT_NAME}
    main.cc
    postprocess.cc
    rknpu2/yolov8.cc
    camera.cpp
    rknn_pool.cpp
    image_process.cpp
   # mpp_decoder.cpp
    mpp_encoder.cpp
    drawing.cpp
    mpprtspdecoder.cpp
    ffstream.cpp
    location.cpp
    
)

target_link_libraries(${PROJECT_NAME}
    fileutils
    imageutils
    imagedrawing    
    ${LIBRKNNRT}
     ${OpenCV_LIBS} 
     ${GST_LIBRARIES}
     ${SDL2_LIBRARIES}
      ${MPP_LIBS}
    ${ZLMEDIAKIT_LIBS}
     ${FFMPEG_LIBRARIES}/libavcodec.so
    ${FFMPEG_LIBRARIES}/libavformat.so
    ${FFMPEG_LIBRARIES}/libavutil.so
     ${Eigen3_LIBS}
)

if (CMAKE_SYSTEM_NAME STREQUAL "Android")
    target_link_libraries(${PROJECT_NAME}
    log
)
endif()

message(STATUS "!!!!!!!!!!!CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)
    target_link_libraries(${PROJECT_NAME} Threads::Threads)
endif()

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${LIBRKNNRT_INCLUDES}
     ${OpenCV_INCLUDE_DIRS}
     ${GST_INCLUDE_DIRS}
     ${SDL2_INCLUDE_DIRS}
     ${FFMPEG_INCLUDE_DIRS}
     ${Eigen3_INCLUDE_DIRS}
    # ${FFMPEG_INCLUDE_DIRS}
)

install(TARGETS ${PROJECT_NAME} DESTINATION .)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../model/bus.jpg DESTINATION model)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../model/coco_80_labels_list.txt DESTINATION model)
file(GLOB RKNN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/../model/*.rknn")
install(FILES ${RKNN_FILES} DESTINATION model)
if(ZLMEDIAKIT_LIBS)
  install(PROGRAMS ${ZLMEDIAKIT_LIBS} DESTINATION lib)
endif()
if(MPP_LIBS)
  #install(TARGETS rknn_yolov5_video_demo DESTINATION ./)
  install(PROGRAMS ${MPP_LIBS} DESTINATION lib)
endif()
