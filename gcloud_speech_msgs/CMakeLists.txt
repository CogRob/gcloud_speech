cmake_minimum_required(VERSION 2.8.3)
project(gcloud_speech_msgs)

find_package(
  catkin REQUIRED
  COMPONENTS actionlib_msgs message_generation std_msgs)

add_message_files(
  DIRECTORY msg
  FILES LinearPcm16Le16000Audio.msg RecognitionHypothesis.msg)

add_action_files(DIRECTORY action FILES SpeechToText.action)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)
