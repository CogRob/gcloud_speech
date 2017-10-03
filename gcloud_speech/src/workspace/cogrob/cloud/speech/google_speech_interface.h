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

#ifndef COGROB_CLOUD_SPEECH_GOOGLE_SPEECH_INTERFACE_H_
#define COGROB_CLOUD_SPEECH_GOOGLE_SPEECH_INTERFACE_H_

#include <string>
#include <vector>

#include "util/status.h"
#include "util/statusor.h"
#include "cogrob/cloud/speech/audio_sample.h"
#include "cogrob/cloud/speech/proto/recognition_result.pb.h"

namespace cogrob {
namespace cloud {
namespace speech {

class GoogleSpeechRecognizerInterface {
 public:
  GoogleSpeechRecognizerInterface() {}
  virtual ~GoogleSpeechRecognizerInterface() {}

  // Starts voice recognition, non blocking, audio_queue and result_queue must
  // be valid until Stop() is called or IsRunning() returns false.
  virtual util::Status StartRecognize(AudioQueue* audio_queue,
      util::SimpleThreadSafeQueue<RecognitionResult>* result_queue,
      const std::vector<std::string>& hints = {}, int max_audio_seconds = 14,
      int max_wait_seconds = 17, int max_alternatives = 10) = 0;

  // TODO(shengye): Add a StartRecognizeBlocking method

  // Test whether the recognizer is still running.
  virtual bool IsRunning() = 0;

  // Return the last result
  virtual util::StatusOr<RecognitionResult> GetLastResult() = 0;

  // Wait until recognizer finishes
  virtual util::Status Wait() = 0;

  // Stop recognition
  virtual util::Status Stop() = 0;
};

}  // namespace speech
}  // namespace cloud
}  // namespace cogrob

#endif  // COGROB_CLOUD_SPEECH_GOOGLE_SPEECH_INTERFACE_H_
