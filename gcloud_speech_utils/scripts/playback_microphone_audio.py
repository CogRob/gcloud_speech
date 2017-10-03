#!/usr/bin/env python
import array
import collections
import pyaudio
import Queue
import time

import rospy
import gcloud_speech_msgs.msg as gcloud_speech_msgs

LinearPcm16Le16000Audio = gcloud_speech_msgs.LinearPcm16Le16000Audio

FRAME_PER_SLICE=1600

_audio_queue = collections.deque()


def RosCallback(msg):
  _audio_queue.append(msg.data)
  rospy.loginfo("Received {} bytes.".format(len(msg.data)))


def PaCallback(in_data, frame_count, time_info, status):
  assert frame_count == FRAME_PER_SLICE

  while True:
    try:
      data = _audio_queue.popleft()
      break
    except:
      time.sleep(.5)

  return (data, pyaudio.paContinue)


def PlaybackMicrophoneAudio():
    rospy.init_node('playback_microphone_audio', anonymous=True)
    rospy.Subscriber(
      "/cogrob/microphone_audio", LinearPcm16Le16000Audio, RosCallback,
      queue_size=10)
    pa = pyaudio.PyAudio()
    stream = pa.open(
      format=pyaudio.paInt16, channels=1, rate=16000, output=True,
      stream_callback=PaCallback, frames_per_buffer=FRAME_PER_SLICE)

    stream.start_stream()
    rospy.spin()
    pa.terminate()


if __name__ == '__main__':
  try:
    PlaybackMicrophoneAudio()
  except rospy.ROSInterruptException:
    pass
