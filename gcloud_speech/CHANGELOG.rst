^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gcloud_speech
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2017-10-06)
------------------
* Adds catkin_INCLUDE_DIRS to include_directories (`#16 <https://github.com/CogRob/gcloud_speech/issues/16>`_)
* Contributors: Shengye Wang

0.0.2 (2017-10-06)
------------------
* Fixes a problem that gflags is not found in the action server node source (`#14 <https://github.com/CogRob/gcloud_speech/issues/14>`_)
* Contributors: Shengye Wang

0.0.1 (2017-10-06)
------------------
* Removes the clicking sound in playback microphone node (`#9 <https://github.com/CogRob/gcloud_speech/issues/9>`_)
* Adds a playback node to gcloud_speech_utils (`#7 <https://github.com/CogRob/gcloud_speech/issues/7>`_)
  * Adds a microphone playback node
  * Use a newer version of record_audio.cc, and adds a playback node to utils
* Adds documents and examples, fixes bugs. (`#6 <https://github.com/CogRob/gcloud_speech/issues/6>`_)
  * Move print_rms exclusively to utils package
  * Implements auto retrying if failed prematurely within 300ms (default)
  * Adds some information in README file
  * Documentation
  * Document update
  * URL better to be link
  * Adds a All-In-One launch file that demos the program
* Adds googleapis/.GIT_STATUS file to record the source googleapis version. (`#5 <https://github.com/CogRob/gcloud_speech/issues/5>`_)
* Adds channel connect and channel check to google_speech (`#3 <https://github.com/CogRob/gcloud_speech/issues/3>`_)
* Initial commit. (`#1 <https://github.com/CogRob/gcloud_speech/issues/1>`_)
* Contributors: Shengye Wang
