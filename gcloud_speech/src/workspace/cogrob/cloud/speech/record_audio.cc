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

#include "cogrob/cloud/speech/record_audio.h"

#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "cogrob/cloud/speech/audio_sample.h"
#include "third_party/portaudio.h"
#include "third_party/glog.h"
#include "third_party/gflags.h"

DEFINE_string(mic, "USB", "Name of the microphone");

using std::unique_ptr;
using std::string;

namespace cogrob {
namespace cloud {
namespace speech {

AudioRecorder::AudioRecorder(AudioQueue* output_queue) {
  queue_ = output_queue;
  StartRecording();
}

AudioRecorder::~AudioRecorder() {
  StopRecording();
}

void AudioRecorder::StartRecording() {
  PaError pa_err = paNoError;

  // Intialize PortAudio
  pa_err = Pa_Initialize();
  if (pa_err != paNoError) {
    LOG(FATAL) << "PortAudio init error: " << Pa_GetErrorText(pa_err);
  }

  // Find the microphone
  PaDeviceIndex num_devices;
  num_devices = Pa_GetDeviceCount();
  if (num_devices < 0) {
      LOG(FATAL) << "Pa_CountDevices returned " << num_devices;
  }
  bool found_mic = false;
  PaDeviceIndex device_index = 0;
  const PaDeviceInfo* device_info;
  for (PaDeviceIndex i = 0; i < num_devices; ++i) {
      device_info = Pa_GetDeviceInfo(i);
      LOG(INFO) << "Device " << device_info->name << " has "
                << device_info->maxInputChannels << " input channels.";
      if (string(device_info->name).find(FLAGS_mic) != string::npos) {
        found_mic = true;
        device_index = i;
        LOG(INFO) << "Use device " << device_info->name;
        break;
      }
  }
  if (!found_mic) {
    LOG(FATAL) << "Can not find device " << FLAGS_mic;
  }

  // Open an audio I/O stream
  PaStreamParameters input_param = {
    .device = device_index,
    .channelCount = 1,
    .sampleFormat = paInt16,
    .suggestedLatency = 0,
    .hostApiSpecificStreamInfo = nullptr
  };

  pa_err = Pa_OpenStream(&pa_stream_, &input_param, nullptr, SAMPLE_RATE,
                         SAMPLES_PER_SLICE, paNoFlag,
                         AudioRecorder::PortAudioCallback, this);

  if (pa_err != paNoError) {
    LOG(FATAL) << "PortAudio open stream error: " << Pa_GetErrorText(pa_err);
  }
  // Start the stream
  Pa_StartStream(pa_stream_);
  if (pa_err != paNoError) {
    LOG(FATAL) << "PortAudio start stream error: " << Pa_GetErrorText(pa_err);
  }

}

void AudioRecorder::StopRecording() {
  PaError pa_err = paNoError;
  Pa_StopStream(pa_stream_);
  if (pa_err != paNoError) {
    LOG(FATAL) << "PortAudio stop stream error: " << Pa_GetErrorText(pa_err);
  }
  Pa_CloseStream(pa_stream_);
  if (pa_err != paNoError) {
    LOG(FATAL) << "PortAudio close stream error: " << Pa_GetErrorText(pa_err);
  }
}

int AudioRecorder::PortAudioCallback(
    const void* input, void* output, unsigned long frame_count,
    const PaStreamCallbackTimeInfo* time_info,
    PaStreamCallbackFlags status_flags, void* user_data) {
  AudioRecorder* recorder = static_cast<AudioRecorder*>(user_data);

  LOG_IF(ERROR, frame_count != SAMPLES_PER_SLICE) << "Callback frame_count is "
      << frame_count << ", which is not " << SAMPLES_PER_SLICE;
  LOG_IF(ERROR, status_flags) << "Callback status flag is " << status_flags;

  std::unique_ptr<AudioSample> audio_sample(new AudioSample());
  audio_sample->resize(frame_count * 2);

  memcpy(audio_sample->data(), input, frame_count * 2);

  recorder->queue_->push(std::move(audio_sample));

  return paContinue;
}

}  // namespace speech
}  // namespace cloud
}  // namespace cogrob
