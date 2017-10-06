cmake_minimum_required(VERSION 2.8.3)
project(gcloud_speech)

find_package(catkin
  REQUIRED COMPONENTS actionlib actionlib_msgs gcloud_speech_msgs grpc roscpp)

catkin_package()

# Generate and compile googleapis.
# The following protoi list is generated using:
# find googleapis/google/api googleapis/google/rpc \
#   googleapis/google/cloud/speech googleapis/google/longrunning \
#   googleapis/google/logging googleapis/google/type -name '*.proto'

set(GOOGLEAPIS_SPEECH_PROTOS
  googleapis/google/api/metric.proto
  googleapis/google/api/monitored_resource.proto
  googleapis/google/api/experimental/experimental.proto
  googleapis/google/api/experimental/authorization_config.proto
  googleapis/google/api/annotations.proto
  googleapis/google/api/system_parameter.proto
  googleapis/google/api/distribution.proto
  googleapis/google/api/http.proto
  googleapis/google/api/servicemanagement/v1/servicemanager.proto
  googleapis/google/api/servicemanagement/v1/resources.proto
  googleapis/google/api/service.proto
  googleapis/google/api/monitoring.proto
  googleapis/google/api/config_change.proto
  googleapis/google/api/billing.proto
  googleapis/google/api/endpoint.proto
  googleapis/google/api/log.proto
  googleapis/google/api/source_info.proto
  googleapis/google/api/usage.proto
  googleapis/google/api/context.proto
  googleapis/google/api/httpbody.proto
  googleapis/google/api/logging.proto
  googleapis/google/api/quota.proto
  googleapis/google/api/auth.proto
  googleapis/google/api/backend.proto
  googleapis/google/api/servicecontrol/v1/operation.proto
  googleapis/google/api/servicecontrol/v1/service_controller.proto
  googleapis/google/api/servicecontrol/v1/check_error.proto
  googleapis/google/api/servicecontrol/v1/log_entry.proto
  googleapis/google/api/servicecontrol/v1/distribution.proto
  googleapis/google/api/servicecontrol/v1/quota_controller.proto
  googleapis/google/api/servicecontrol/v1/metric_value.proto
  googleapis/google/api/control.proto
  googleapis/google/api/consumer.proto
  googleapis/google/api/documentation.proto
  googleapis/google/api/label.proto
  googleapis/google/rpc/error_details.proto
  googleapis/google/rpc/code.proto
  googleapis/google/cloud/speech/v1/cloud_speech.proto
  googleapis/google/cloud/speech/v1_1beta1/cloud_speech.proto
  googleapis/google/cloud/speech/v1beta1/cloud_speech.proto
  googleapis/google/longrunning/operations.proto
  googleapis/google/logging/type/http_request.proto
  googleapis/google/logging/type/log_severity.proto
  googleapis/google/logging/v2/log_entry.proto
  googleapis/google/logging/v2/logging_config.proto
  googleapis/google/logging/v2/logging.proto
  googleapis/google/logging/v2/logging_metrics.proto
  googleapis/google/type/dayofweek.proto
  googleapis/google/type/date.proto
  googleapis/google/type/postal_address.proto
  googleapis/google/type/timeofday.proto
  googleapis/google/type/latlng.proto
  googleapis/google/type/color.proto
  googleapis/google/type/money.proto
)

# Compile googleapis.
generate_proto(googleapis_speech_proto GRPC ${GOOGLEAPIS_SPEECH_PROTOS})

# Compile the library in workspace.
generate_proto(cogrob_gcloud_speech_proto
  SRC_BASE src/workspace
  src/workspace/cogrob/cloud/speech/proto/recognition_result.proto)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(src/workspace)
include_directories(src/googleapis_include)

add_library(gcloud_speech_lib
  src/workspace/util/statusor.cc
  src/workspace/util/status.cc
  src/workspace/cogrob/cloud/basic/defaults.cc
  src/workspace/cogrob/cloud/speech/google_speech.cc
)
add_dependencies(gcloud_speech_lib googleapis_speech_proto)
target_link_libraries(
  gcloud_speech_lib cogrob_gcloud_speech_proto googleapis_speech_proto
  glog gflags)

add_executable(gcloud_speech_action_node
  src/gcloud_speech_action_node.cc src/speech_to_text_action_handler.cc)
target_link_libraries(gcloud_speech_action_node
  ${catkin_LIBRARIES} gcloud_speech_lib)

option(GCLOUD_SPEECH_LOCAL_MAIN "GCLOUD_SPEECH_LOCAL_MAIN" OFF)
if(GCLOUD_SPEECH_LOCAL_MAIN)
  add_library(gcloud_speech_record_audio_lib
    src/workspace/cogrob/cloud/speech/record_audio.cc
  )
  target_link_libraries(gcloud_speech_record_audio_lib portaudio)

  add_executable(speech_local_main
    src/workspace/cogrob/cloud/speech/speech_local_main.cc)
  target_link_libraries(speech_local_main
    gcloud_speech_record_audio_lib gcloud_speech_lib gflags glog)
endif()
