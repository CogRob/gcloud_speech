#include "speech_to_text_action_handler.h"

#include "third_party/gflags.h"
#include "third_party/glog.h"
#include "util/statusor.h"

namespace speech = ::cogrob::cloud::speech;

using SpeechToTextSimpleActionServer =
    actionlib::SimpleActionServer<gcloud_speech_msgs::SpeechToTextAction>;

DEFINE_int32(speech_fail_prematurely_retry_cutoff_msec, 300,
    "Cutoff duration to allow retrying if recognition failed prematurely.");

namespace gcloud_speech {

SpeechToTextActionHandler::SpeechToTextActionHandler(
    speech::GoogleSpeechRecognizerInterface* recognizer,
    SpeechToTextSimpleActionServer* simple_action_server) {
  recognizer_ = recognizer;
  simple_action_server_ = simple_action_server;
}


void SpeechToTextActionHandler::AudioMsgCallback(
    const gcloud_speech_msgs::LinearPcm16Le16000Audio::ConstPtr& msg) {
  if (msg->data.size() & 1 != 0) {
      LOG(ERROR) << "Size of data in LinearPcm16Le16000Audio is not "
                 << "multiple of 2. Discarding sample.";
      DCHECK(false);
      return;
  }
  std::unique_ptr<speech::AudioSample> audio_sample(
      new speech::AudioSample(msg->data));
  audio_queue_.push(std::move(audio_sample));
}

void SpeechToTextActionHandler::ExecuteSpeechToTextAction(
    const gcloud_speech_msgs::SpeechToTextGoalConstPtr& goal) {
  audio_queue_.clear();
  result_queue_.clear();
  recognizer_->Stop();

  int max_audio_seconds = goal->listen_duration_sec;
  if (max_audio_seconds == 0) {
    max_audio_seconds = 14;
  }
  int max_wait_seconds = goal->max_recognition_duration_sec;
  if (max_wait_seconds == 0) {
    max_wait_seconds = max_audio_seconds + 2;
  }

  bool interim_results = !goal->suppress_interim_results;

  // This is the result we will publish.
  gcloud_speech_msgs::SpeechToTextResult result_msg;

  int retry_time_left = 2;
  std::chrono::system_clock::time_point retry_deadline =
    std::chrono::system_clock::now() +
    std::chrono::milliseconds(FLAGS_speech_fail_prematurely_retry_cutoff_msec);

  while (retry_time_left > 0
         && std::chrono::system_clock::now() < retry_deadline) {
    LOG(INFO) << "Start recognize.";
    recognizer_->StartRecognize(&audio_queue_, &result_queue_, goal->hints,
        max_audio_seconds, max_wait_seconds, goal->max_alternatives);

    while (recognizer_->IsRunning() &&
        !simple_action_server_->isPreemptRequested()) {
      util::StatusOr<speech::RecognitionResult> result
          = result_queue_.blocking_pop(100);
      if (result.ok()) {
        // Processes the result and post some feedback.
        LOG(INFO) << "Result: " << result.ValueOrDie().ShortDebugString();

        if (result.ValueOrDie().is_final() or interim_results) {
          gcloud_speech_msgs::SpeechToTextFeedback feedback_msg;
          for (const auto& candidate: result.ValueOrDie().candidates()) {
            gcloud_speech_msgs::RecognitionHypothesis hypothesis;
            hypothesis.transcript = candidate.transcript();
            hypothesis.confidence = candidate.confidence();
            feedback_msg.hypotheses.push_back(hypothesis);
          }
          feedback_msg.is_portion_final = result.ValueOrDie().is_final();
          feedback_msg.stability = result.ValueOrDie().stability();
          simple_action_server_->publishFeedback(feedback_msg);
        }

        if (result.ValueOrDie().is_final()) {
          if (result.ValueOrDie().candidates().size() > 0) {
            result_msg.transcript +=
                " " + result.ValueOrDie().candidates()[0].transcript();
          }
        }
      }
    }

    recognizer_->Stop();

    if (recognizer_->GetLastResult().ok()) {
      // If there is no error, we can quit retrying.
      break;
    }
    // Decreate retry counter so we don't retry too many times.
    --retry_time_left;
  }

  util::StatusOr<speech::RecognitionResult> last_result =
      recognizer_->GetLastResult();
  if (!last_result.ok()) {
    result_msg.is_error = true;
    result_msg.error_info = last_result.status().error_message();
  }

  if (simple_action_server_->isPreemptRequested()) {
    simple_action_server_->setPreempted(result_msg);
  } else {
    simple_action_server_->setSucceeded(result_msg);
  }
}

}  // namespace gcloud_speech
