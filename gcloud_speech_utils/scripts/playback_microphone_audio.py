#!/usr/bin/env python
import array
import collections
import pyaudio
import time

import rospy
import gcloud_speech_msgs.msg as gcloud_speech_msgs

LinearPcm16Le16000Audio = gcloud_speech_msgs.LinearPcm16Le16000Audio

MSG_FRAME_PER_SLICE = 1600

_audio_queue = collections.deque()


def RosCallback(msg):
  _audio_queue.append(msg.data)
  rospy.loginfo("Received {} bytes.".format(len(msg.data)))


def PaCallback(in_data, frame_count, time_info, status):
  assert frame_count == MSG_FRAME_PER_SLICE * 2

  while True:
    try:
      # For some reason putting in 10 chunks per second to PyAudio introduces a
      # lot of clicking sound. This hack only combines two chunks so that only
      # PyAudio only needs to handle 5 chunks per seconds, and the clicking
      # sound went alway.
      data_1 = _audio_queue.popleft()
      data_2 = _audio_queue.popleft()
      data = data_1 + data_2
      break
    except:
      time.sleep(1)

  return (data, pyaudio.paContinue)


def PlaybackMicrophoneAudio():
    rospy.init_node('playback_microphone_audio', anonymous=True)
    rospy.Subscriber(
      "/cogrob/microphone_audio", LinearPcm16Le16000Audio, RosCallback,
      queue_size=10)
    pa = pyaudio.PyAudio()
    stream = pa.open(
      format=pyaudio.paInt16, channels=1, rate=16000, output=True,
      stream_callback=PaCallback, frames_per_buffer=MSG_FRAME_PER_SLICE * 2)

    stream.start_stream()
    rospy.spin()
    pa.terminate()


if __name__ == '__main__':
  try:
    PlaybackMicrophoneAudio()
  except rospy.ROSInterruptException:
    pass
