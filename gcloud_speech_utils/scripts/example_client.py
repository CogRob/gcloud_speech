#! /usr/bin/env python
from __future__ import print_function

import signal
import sys
import time

import actionlib
import actionlib_msgs.msg as actionlib_msgs
import rospy

import gcloud_speech_msgs.msg as gcloud_speech_msgs

GoalStatus = actionlib_msgs.GoalStatus

def SpeechToTextSimpleExampleClient():
  def DoneCallback(state, result):
    print("\n\nDone, state {}, result:\n{}\n".format(state,result))


  def ActiveCallback():
    print("The goal is now active.\n")


  def FeedbackCallback(feedback):
    print("{}\n".format(feedback))


  global sigint_received
  sigint_received = False

  def SignalIntHandler(signal, frame):
    print("Received SIGINT.")
    global sigint_received
    sigint_received = True


  signal.signal(signal.SIGINT, SignalIntHandler)

  client = actionlib.SimpleActionClient(
    "/cogrob/speech_to_text", gcloud_speech_msgs.SpeechToTextAction)
  client.wait_for_server()

  goal = gcloud_speech_msgs.SpeechToTextGoal()
  client.send_goal(goal, done_cb=DoneCallback, active_cb=ActiveCallback,
                   feedback_cb=FeedbackCallback)

  while client.get_state() in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
    time.sleep(1)
    if sigint_received:
      break

  if sigint_received:
    client.cancel_goal()

  client.wait_for_result()


if __name__ == '__main__':
  try:
    rospy.init_node("gcloud_speech_example_client", anonymous=True)
    SpeechToTextSimpleExampleClient()
  except rospy.ROSInterruptException:
    print("Program interrupted before completion.", file=sys.stderr)
