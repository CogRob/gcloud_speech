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

#include <cmath>

#include "ros/ros.h"
#include "gcloud_speech_msgs/LinearPcm16Le16000Audio.h"

#include "cogrob/cloud/speech/record_audio.h"
#include "third_party/gflags.h"
#include "third_party/glog.h"
#include "util/simple_thread_safe_queue.h"
#include "util/statusor.h"

using gcloud_speech_msgs::LinearPcm16Le16000Audio;

namespace speech = ::cogrob::cloud::speech;
using speech::AudioSample;

DEFINE_int32(mic_fatal_timeout_msec, 2000,
    "Timeout (ms) to terminate the program if no sample is available.");

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();
  gflags::SetUsageMessage(
      "Record audio and publish as LinearPcm16Le16000Audio message.");
  gflags::SetVersionString("1.0.0");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "record_audio");
  ros::NodeHandle node_handle;

  util::SimpleThreadSafeQueue<std::unique_ptr<AudioSample>> audio_queue;
  speech::AudioRecorder recorder(&audio_queue);
  ros::Publisher mic_pub =
      node_handle.advertise<gcloud_speech_msgs::LinearPcm16Le16000Audio>
      ("/cogrob/microphone_audio", 10);

  while (ros::ok()) {
    util::StatusOr<std::unique_ptr<AudioSample>> queue_result = std::move(
        audio_queue.blocking_pop(FLAGS_mic_fatal_timeout_msec));
    if (!queue_result.ok()) {
      LOG(FATAL) << "Getting audio from microphone timed out.";
    }
    std::unique_ptr<AudioSample> sample = std::move(
        queue_result.ConsumeValueOrDie());
    LinearPcm16Le16000Audio audio_msg;
    audio_msg.data = *sample;
    mic_pub.publish(audio_msg);
  }
}
