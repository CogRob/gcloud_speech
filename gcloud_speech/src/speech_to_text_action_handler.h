#ifndef SPEECH_TO_TEXT_ACTION_HANDLER_H_
#define SPEECH_TO_TEXT_ACTION_HANDLER_H_

#include "actionlib/server/simple_action_server.h"
#include "gcloud_speech_msgs/SpeechToTextAction.h"
#include "gcloud_speech_msgs/RecognitionHypothesis.h"
#include "gcloud_speech_msgs/LinearPcm16Le16000Audio.h"

#include "cogrob/cloud/speech/audio_sample.h"
#include "cogrob/cloud/speech/google_speech_interface.h"
#include "third_party/glog.h"
#include "util/simple_thread_safe_queue.h"

namespace gcloud_speech {

class SpeechToTextActionHandler {
 public:
  // Constructor: recognizer is a GoogleSpeechRecognizer that will be used to
  // contact Google Speech API and do speech recognition, simple_action_server
  // is a SimpleActionServer handle that interacts with client.
  SpeechToTextActionHandler(
      cogrob::cloud::speech::GoogleSpeechRecognizerInterface* recognizer,
      actionlib::SimpleActionServer<gcloud_speech_msgs::SpeechToTextAction>*
          simple_action_server);

  // ROS subscriber callback for LinearPcm16Le16000Audio.
  void AudioMsgCallback(
      const gcloud_speech_msgs::LinearPcm16Le16000Audio::ConstPtr& msg);

  // ROS callback for SimpleActionServer.
  void ExecuteSpeechToTextAction(
      const gcloud_speech_msgs::SpeechToTextGoalConstPtr& goal);

 private:
  // audio_queue_ contains audio samples, and will be populated by
  // AudioMsgCallback function.
  cogrob::cloud::speech::AudioQueue audio_queue_;

  // result_queue_ contains results from recognizer_.
  util::SimpleThreadSafeQueue<cogrob::cloud::speech::RecognitionResult>
      result_queue_;

  cogrob::cloud::speech::GoogleSpeechRecognizerInterface* recognizer_;
  actionlib::SimpleActionServer<gcloud_speech_msgs::SpeechToTextAction>*
      simple_action_server_;
};

}  // namespace gcloud_speech

#endif  // SPEECH_TO_TEXT_ACTION_HANDLER_H_
