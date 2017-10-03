#include <functional>
#include <memory>
#include <mutex>

#include "actionlib/server/simple_action_server.h"
#include "gcloud_speech_msgs/SpeechToTextAction.h"
#include "gcloud_speech_msgs/RecognitionHypothesis.h"
#include "gcloud_speech_msgs/LinearPcm16Le16000Audio.h"

#include "cogrob/cloud/basic/defaults.h"
#include "cogrob/cloud/speech/google_speech.h"
#include "cogrob/cloud/speech/google_speech_interface.h"

#include "speech_to_text_action_handler.h"

namespace speech = ::cogrob::cloud::speech;

using gcloud_speech_msgs::LinearPcm16Le16000Audio;
using SpeechToTextSimpleActionServer =
    actionlib::SimpleActionServer<gcloud_speech_msgs::SpeechToTextAction>;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  cogrob::cloud::PrepareGoogleCloudCredentials();
  grpc_init();

  ros::init(argc, argv, "gcloud_speech_action_server");
  ros::NodeHandle ros_node_handle;

  std::mutex action_handler_mutex;
  std::unique_ptr<gcloud_speech::SpeechToTextActionHandler> action_handler;

  std::unique_ptr<speech::GoogleSpeechRecognizerInterface> recognizer(
      new speech::GoogleSpeechRecognizer());

  std::function<void (const SpeechToTextSimpleActionServer::GoalConstPtr &)>
      exec_callback = [&action_handler_mutex, &action_handler]
      (const SpeechToTextSimpleActionServer::GoalConstPtr& goal) {
          std::lock_guard<std::mutex> lock(action_handler_mutex);
          if (action_handler) {
            action_handler->ExecuteSpeechToTextAction(goal);
          }
      };

  SpeechToTextSimpleActionServer action_server(
      ros_node_handle, "/cogrob/speech_to_text", exec_callback, false);

  {
    // Limit the scope of the lock.
    std::lock_guard<std::mutex> lock(action_handler_mutex);
    action_handler.reset(new gcloud_speech::SpeechToTextActionHandler(
        recognizer.get(), &action_server));
  }

  ros::Subscriber audio_subscriber =
      ros_node_handle.subscribe<LinearPcm16Le16000Audio>(
      "/cogrob/microphone_audio", 10,
      [&action_handler] (const LinearPcm16Le16000Audio::ConstPtr& msg) {
        action_handler->AudioMsgCallback(msg);
      });

  action_server.start();
  LOG(INFO) << "SpeechToTextSimpleActionServer started.";
  ros::spin();
  return 0;
}
