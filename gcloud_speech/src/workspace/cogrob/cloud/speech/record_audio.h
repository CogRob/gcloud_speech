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

#ifndef COGROB_CLOUD_SPEECH_RECORD_AUDIO_H_
#define COGROB_CLOUD_SPEECH_RECORD_AUDIO_H_

#include "portaudio.h"

#include <atomic>
#include <memory>
#include <thread>

#include "util/simple_thread_safe_queue.h"
#include "cogrob/cloud/speech/audio_sample.h"

namespace cogrob {
namespace cloud {
namespace speech {

class AudioRecorder {
 public:
  explicit AudioRecorder(AudioQueue* output_queue);
  virtual ~AudioRecorder();
 private:
  AudioQueue* queue_;
  PaStream* pa_stream_ = nullptr;
  void StartRecording();
  void StopRecording();
  static int PortAudioCallback(
      const void* input, void* output, unsigned long frame_count,
      const PaStreamCallbackTimeInfo* time_info,
      PaStreamCallbackFlags status_flags, void* user_data);
};

}  // namespace speech
}  // namespace cloud
}  // namespace cogrob

#endif  // COGROB_CLOUD_SPEECH_RECORD_AUDIO_H_
