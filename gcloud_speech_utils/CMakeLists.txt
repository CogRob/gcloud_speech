cmake_minimum_required(VERSION 2.8.3)
project(gcloud_speech_utils)

if(CMAKE_VERSION VERSION_LESS "3.1")
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
  else()
    message(SEND_ERROR "Error: Can not enable C++ 11 for gRPC")
  endif()
else()
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(catkin
  REQUIRED COMPONENTS actionlib actionlib_msgs gcloud_speech_msgs roscpp rospy)

catkin_package()

include_directories(src/workspace)

add_library(gcloud_speech_utils_record_audio_lib
  src/workspace/cogrob/cloud/speech/record_audio.cc
)
target_link_libraries(gcloud_speech_utils_record_audio_lib portaudio)

add_executable(gcloud_speech_utils_print_rms
  src/workspace/cogrob/cloud/speech/print_rms.cc)
target_link_libraries(gcloud_speech_utils_print_rms
  gcloud_speech_utils_record_audio_lib gflags glog)

add_executable(record_microphone_audio src/record_microphone_audio.cc)
target_link_libraries(record_microphone_audio ${catkin_LIBRARIES}
  gcloud_speech_utils_record_audio_lib gflags glog)