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

#include <grpc++/grpc++.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <string>

#include "cogrob/cloud/basic/defaults.h"
#include "third_party/gflags.h"
#include "third_party/glog.h"
#include "util/simple_thread_safe_queue.h"

#include "cogrob/cloud/speech/audio_sample.h"
#include "cogrob/cloud/speech/record_audio.h"
#include "cogrob/cloud/speech/google_speech.h"
#include "cogrob/cloud/speech/google_speech_interface.h"

DEFINE_bool(early_interrupt, false, "Whether to early interrupt the request");

namespace speech = ::cogrob::cloud::speech;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  cogrob::cloud::PrepareGoogleCloudCredentials();
  grpc_init();

  util::SimpleThreadSafeQueue<speech::RecognitionResult> result_queue;
  speech::AudioQueue audio_queue;
  speech::AudioRecorder recorder(&audio_queue);
  std::unique_ptr<speech::GoogleSpeechRecognizerInterface> recognizer(
      new speech::GoogleSpeechRecognizer());

  while (true) {
    LOG(INFO) << "Main thread Running";
    audio_queue.clear();
    result_queue.clear();
    std::cerr << "Start recognize." << std::endl;
    recognizer->StartRecognize(&audio_queue, &result_queue, {"CogRob"}, 15, 18);
    std::chrono::system_clock::time_point early_interrupt_deadline =
        std::chrono::system_clock::now() + std::chrono::seconds(5);
    while (recognizer->IsRunning()) {
      if (FLAGS_early_interrupt &&
          std::chrono::system_clock::now() > early_interrupt_deadline) {
        recognizer->Stop();
      }
      util::StatusOr<speech::RecognitionResult> result
          = result_queue.blocking_pop(100);
      if (result.ok()) {
        std::cerr << "----------------------------------------" << std::endl;
        std::cerr << result.ConsumeValueOrDie().DebugString() << std::endl;
      }
    }
  }

  LOG(ERROR) << "Main thread about to finish.";
}
