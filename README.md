# `gcloud_speech`: Google Cloud Speech API ROS Integration

These packages integrate
[Google Cloud Speech API](https://cloud.google.com/speech/) into
[ROS](http://www.ros.org/) ecosystem.

## Usage

### `gcloud_speech_msgs`: ROS Message Definitions

`gcloud_speech_msgs` defines following messages:

* `gcloud_speech_msgs/LinearPcm16Le16000Audio` represents raw audio buffer in
  Linear PCM format, 16 bits, little endian, sample rate 16,000 Hz. It could be
  generated/converted by any other nodes.
* `gcloud_speech_msgs/RecognitionHypothesis` represents a recognition
  hypothesis. It is a pair of transcript and confidence score.
* `gcloud_speech_msgs/SpeechToTextAction` and other action server messages
  defines a `actionlib` interface to interact with Google Cloud Speech API.


### `gcloud_speech`: Google Cloud Speech API ActionServer

`gcloud_speech/gcloud_speech_action_node` is a SimpleActionServer node. It
could only serve one goal at a time (theoretically it could serve multiple
goals, but we don't have an immediate plan for this). Newer goal preempts older
goal. A goal specifies hints, max listening time, max execution time, and some
other parameters. And the action server will start streaming results from Google
Cloud Speech API through action feedback. A goal can be preempted/canceled
anytime, and the result will return a single transcript. We highly recommend
relying on feedback to get recognition results.

`gcloud_speech_action_node` serves the action server at
`/cogrob/speech_to_text` with type `gcloud_speech_msgs/SpeechToTextAction`.
It listens to `/cogrob/microphone_audio` audio topic with type
`gcloud_speech_msgs/LinearPcm16Le16000Audio`.

To start `gcloud_speech_action_node`, a few command line flags must be set
correctly:
```
--gcloud_cred=/path/to/your/google_cloud_credential.json
--grpc_roots=/path/to/grpc_roots.pem
```
The `grpc_roots.pem` file can be downloaded from
[here](https://github.com/grpc/grpc/blob/master/etc/roots.pem). We have also
included a copy in `gcloud_speech_utils` package under `assets` directory.

#### For Developers

We provide a `speech_local_main` program that does not rely on ROS. It uses
`portaudio` (`portaudio19-dev`) to capture audio and prints recognition result
to the screen. To build this program, enable set `-DGCLOUD_SPEECH_LOCAL_MAIN=ON`
when invoking cmake (`catkin build --cmake-args -DGCLOUD_SPEECH_LOCAL_MAIN=ON`).
Command line flag `--mic=MicrophoneName` sets the microphone the program uses.

### `gcloud_speech_utils`: Utilities and Examples
This package provides a `record_microphone_audio` node that publishes
`LinearPcm16Le16000Audio`. By default it looks for USB microphones. You can add
`--mic=MicrophoneName` to override this option. The argument only need to
partially match of the micrphone name seen by portaudio. The
`playback_microphone_audio.py` script enables monitoring the microphone topic.
Due to limitations of PortAudio and ROS, there is noticeable latency.

This package also provides a `example_client.py` program that interacts with the
ActionServer.

You may also find some useful ROS launch files in this package.

Everything is `gcloud_speech_utils` package is experimental and for demostration
purpose only. We recommend you NOT to use them in critical environment. Use
these software at your own risk.

### Google Cloud Console

Please visit https://console.cloud.google.com to add a project and create
service accounts for the users under "IAM & admin". We recommend one service
account for each machine. You will be able to create keys (in JSON format) in
the same console.

You *must* enable Speech API before you can use it. We have experienced no error
message shown for Speech API not enabled error in the past. To enabled Speech
API, search "Speech" in the search bar.

## License

This package is licensed under the 3-clause BSD License.

Copyright &copy;(2017) The Regents of the University of California, All rights
reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
* Neither the name of the University of California nor the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OF THE UNIVERSITY OF CALIFORNIA BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
