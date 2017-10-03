// Copyright (c) 2017, The Regents of the University of California
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
// * Neither the name of the University of California nor the
//   names of its contributors may be used to endorse or promote products
//   derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OF THE UNIVERSITY OF CALIFORNIA
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef COGROB_CLOUD_SPEECH_GOOGLE_SPEECH_H_
#define COGROB_CLOUD_SPEECH_GOOGLE_SPEECH_H_

#include <grpc++/grpc++.h>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <string>
#include <vector>

#include "cogrob/cloud/speech/google_speech_interface.h"
#include "cogrob/cloud/speech/proto/recognition_result.pb.h"
#include "google/cloud/speech/v1/cloud_speech.grpc.pb.h"

namespace cogrob {
namespace cloud {
namespace speech {

class GoogleSpeechRecognizer : public GoogleSpeechRecognizerInterface {
 public:
  GoogleSpeechRecognizer();
  ~GoogleSpeechRecognizer() override;

  // Starts voice recognition, non blocking, audio_queue and result_queue must
  // be valid until Stop() is called or IsRunning() returns false.
  util::Status StartRecognize(AudioQueue* audio_queue,
      util::SimpleThreadSafeQueue<RecognitionResult>* result_queue,
      const std::vector<std::string>& hints, int max_audio_seconds,
      int max_wait_seconds, int max_alternatives) override;

  // Tests whether the recognizer is still running.
  bool IsRunning() override;

  // Returns the last result.
  util::StatusOr<RecognitionResult> GetLastResult() override;

  // Waits until recognizer finishes.
  util::Status Wait() override;

  // Stops recognition.
  util::Status Stop() override;

 private:
  // A private function runs in a seperate thread
  void RecognitionThread(AudioQueue* audio_queue,
      util::SimpleThreadSafeQueue<RecognitionResult>* result_queue,
      const std::vector<std::string>& hints, int max_audio_seconds,
      int max_wait_seconds, int max_alternatives);

  std::mutex general_mutex_;
  std::shared_ptr<grpc::ChannelInterface> channel_;
  std::unique_ptr<::google::cloud::speech::v1::Speech::Stub> gspeech_stub_;
  std::atomic_bool stop_flag_ {false};
  std::atomic_bool done_flag_ {false};
  std::unique_ptr<std::thread> thread_;
  util::StatusOr<RecognitionResult> latest_result_;
};

}  // namespace speech
}  // namespace cloud
}  // namespace cogrob

#endif  // COGROB_CLOUD_SPEECH_GOOGLE_SPEECH_H_
