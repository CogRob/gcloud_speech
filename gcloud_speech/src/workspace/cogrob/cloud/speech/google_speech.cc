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

#include "cogrob/cloud/speech/google_speech.h"

#include <grpc++/grpc++.h>
#include <grpc/impl/codegen/connectivity_state.h>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "google/cloud/speech/v1/cloud_speech.grpc.pb.h"
#include "third_party/gflags.h"
#include "third_party/glog.h"

DEFINE_int32(grpc_speech_connect_timeout_secs, 5,
             "Timeout (seconds) to connect to gRPC service.");

DEFINE_int32(gspeech_wait_input_timeout_msecs, 100,
             "Timeout (seconds) to wait for input samples.");

namespace gspeech = ::google::cloud::speech::v1;

using gspeech::RecognitionConfig;
using gspeech::StreamingRecognizeRequest;
using gspeech::StreamingRecognizeResponse;

using std::string;
using std::unique_ptr;
using util::Status;
using util::StatusOr;

namespace cogrob {
namespace cloud {
namespace speech {

GoogleSpeechRecognizer::GoogleSpeechRecognizer() {
  std::lock_guard<std::mutex> lock(general_mutex_);
  // Creates a stub connected to the speech service.
  channel_ = grpc::CreateChannel(
      "speech.googleapis.com", grpc::GoogleDefaultCredentials());
  gspeech_stub_ = std::move(gspeech::Speech::NewStub(channel_));

  // Resets status.
  latest_result_ = Status(
      util::error::FAILED_PRECONDITION, "Recognizer not yet started.");
}

GoogleSpeechRecognizer::~GoogleSpeechRecognizer() {
  Stop();
}

Status GoogleSpeechRecognizer::StartRecognize(AudioQueue* audio_queue,
    util::SimpleThreadSafeQueue<RecognitionResult>* result_queue,
    const std::vector<string>& hints, int max_audio_seconds,
    int max_wait_seconds, int max_alternatives) {
  // This method is not re-entryable.
  std::lock_guard<std::mutex> lock(general_mutex_);

  if (thread_) {
    return Status(
        util::error::ALREADY_EXISTS, "Recognizer is already running.");
  }

  // Starts a new thread.
  stop_flag_.store(false);
  done_flag_.store(false);
  latest_result_ = Status(util::error::UNAVAILABLE,
      "Recognizer just started, nothing received yet.");
  // Hints is passed by value because it is used in a new thread.
  thread_.reset(new std::thread([
      this, audio_queue, result_queue, hints, max_audio_seconds,
      max_wait_seconds, max_alternatives] {
        RecognitionThread(audio_queue, result_queue, hints, max_audio_seconds,
                          max_wait_seconds, max_alternatives);
      }));
  return Status::OK;
}

StatusOr<RecognitionResult> GoogleSpeechRecognizer::GetLastResult() {
  // Returns the latest result.
  std::lock_guard<std::mutex> lock(general_mutex_);
  return latest_result_;
}

// Waits until recognizer finishes.
Status GoogleSpeechRecognizer::Wait() {
  // Methods are not re-entryable.
  std::lock_guard<std::mutex> lock(general_mutex_);
  if (thread_) {
    thread_->join();
    thread_.reset(nullptr);
  }
  return Status::OK;
}

// Stops the recognizer.
Status GoogleSpeechRecognizer::Stop() {
  // Methods are not re-entryable.
  std::lock_guard<std::mutex> lock(general_mutex_);

  // If the thread is still running, set the stop flag and wait for it to stop.
  if (thread_) {
    stop_flag_.store(true);
    thread_->join();
    thread_.reset(nullptr);
  }
  return Status::OK;
}

// Test whether the recognizer is still running.
bool GoogleSpeechRecognizer::IsRunning() {
  // Methods are not re-entryable.
  std::lock_guard<std::mutex> lock(general_mutex_);

  // If the thread is stopped, join the thread and destories the thread object.
  if (done_flag_.load() && thread_) {
    thread_->join();
    thread_.reset(nullptr);
  }

  if (thread_) {
    return true;
  }
  return false;
}

// This runs in a seperate thread
void GoogleSpeechRecognizer::RecognitionThread(AudioQueue* audio_queue,
    util::SimpleThreadSafeQueue<RecognitionResult>* result_queue,
    const std::vector<string>& hints, int max_audio_seconds,
    int max_wait_seconds, int max_alternatives) {

  // If any gRPC failure happens in the, this flags will be set and will exit
  // the thread loop.
  bool fail_flag = false;

  if (max_audio_seconds < 1) {
    latest_result_ = Status(
        util::error::FAILED_PRECONDITION,
        "max_audio_seconds must be greater than 0");
    LOG(ERROR) << latest_result_.status();
    fail_flag = true;
  }

  if (max_audio_seconds >= 65) {
    latest_result_ = Status(
        util::error::FAILED_PRECONDITION,
        "max_audio_seconds must be less than 65");
    LOG(ERROR) << latest_result_.status();
    fail_flag = true;
  }

  if (max_wait_seconds <= max_audio_seconds) {
    latest_result_ = Status(
        util::error::FAILED_PRECONDITION,
        "max_wait_seconds must be greater than max_audio_seconds.");
    LOG(ERROR) << latest_result_.status();
    fail_flag = true;
  }

  if (fail_flag) {
    done_flag_.store(true);
    LOG(ERROR) << "There are some errors on preconditions, "
               << "finishing RecognitionThread.";
    return;
  }

  LOG(INFO) << "RecognitionThread started, will listen for "
            << max_audio_seconds << " seconds, and will return in "
            << max_wait_seconds << " seconds.";

  // The deadline that the thread must return.
  std::chrono::system_clock::time_point deadline
    = std::chrono::system_clock::now() + std::chrono::seconds(max_wait_seconds);
  double left_audio_time = max_audio_seconds;

  // gRPC request handles.
  grpc::ClientContext context;
  grpc::CompletionQueue completion_queue;
  // Sequence number in the completion_queue, increase before use.
  uintptr_t cq_seq = 0;

  // After this point, we must drain the CompletionQueue before the can destory
  // the instance.

  // Deadline to wait to connect and channel and for AsyncStreamingRecognize
  // (stream creation) to return, if the time is exceed, we will give up.
  std::chrono::system_clock::time_point stream_connect_deadline =
      std::chrono::system_clock::now()
      + std::chrono::seconds(FLAGS_grpc_speech_connect_timeout_secs);

  // If the channel is in IDLE and try_to_connect is set to true, try to
  // connect. If connection timeout, give up.
  channel_->GetState(true);
  if (!channel_->WaitForConnected(stream_connect_deadline)) {
    latest_result_ = Status(util::error::ABORTED,
                            "gRPC error: Channel connection took too long.");
    LOG(ERROR) << latest_result_.status();
    fail_flag = true;
  }

  // Check the channel state for debugging purpose.
  string channel_state = "";
  grpc_connectivity_state channel_state_enum = channel_->GetState(false);
  switch (channel_state_enum) {
    case GRPC_CHANNEL_INIT:
      channel_state = "GRPC_CHANNEL_INIT";
      break;
    case GRPC_CHANNEL_IDLE:
      channel_state = "GRPC_CHANNEL_IDLE";
      break;
    case GRPC_CHANNEL_CONNECTING:
      channel_state = "GRPC_CHANNEL_CONNECTING";
      break;
    case GRPC_CHANNEL_READY:
      channel_state = "GRPC_CHANNEL_READY";
      break;
    case GRPC_CHANNEL_TRANSIENT_FAILURE:
      channel_state = "GRPC_CHANNEL_TRANSIENT_FAILURE";
      break;
    case GRPC_CHANNEL_SHUTDOWN:
      channel_state = "GRPC_CHANNEL_SHUTDOWN";
      break;
    default:
      CHECK(false) << "Unknown channel state: " << channel_state_enum;
      break;
  }

  if (channel_state_enum == GRPC_CHANNEL_READY) {
    LOG(INFO) << "Channel state is " << channel_state;
  } else {
    LOG(ERROR) << "Channel state is " << channel_state;
  }

  if (fail_flag) {
    done_flag_.store(true);
    LOG(ERROR) << "There are some errors on gRPC channels, "
               << "finishing RecognitionThread.";
    return;
  }

  // Starts a async call if channel connection is successful.
  auto /* std::unique_ptr<ClientAsyncReaderWriterInterface> */ streamer
      = gspeech_stub_->AsyncStreamingRecognize(
          &context, &completion_queue, reinterpret_cast<void*>(++cq_seq));
  uintptr_t stream_cq_seq = cq_seq;
  LOG(INFO) << "Start a call to Google Speech gRPC server, cq_seq: "
            << stream_cq_seq;

  // Blocks until the creation of the stream is done, we cannot start writing
  // until that happens.
  // See: https://github.com/GoogleCloudPlatform/cpp-docs-samples/issues/16
  // Result from CompletionQueue::AsyncNext
  void* stream_cq_tag = nullptr;
  bool stream_cq_ok = false;
  // Read from the completion_queue with some timeout
  grpc::CompletionQueue::NextStatus stream_cq_state
      = completion_queue.AsyncNext(&stream_cq_tag, &stream_cq_ok,
                                   stream_connect_deadline);
  if (stream_cq_state == grpc::CompletionQueue::GOT_EVENT) {
    if (!stream_cq_ok) {
      latest_result_ = Status(util::error::ABORTED,
                              "gRPC error: Stream failed to create.");
      LOG(ERROR) << latest_result_.status();
      fail_flag = true;
    }

    // stream_cq_ok is true here.
    if (stream_cq_tag == reinterpret_cast<void*>(stream_cq_seq)) {
      // This is the expected condition.
      LOG(INFO) << "gRPC created stream, tag is "
                << reinterpret_cast<uintptr_t>(stream_cq_tag);
    } else {
      latest_result_ = Status(util::error::ABORTED,
          "gRPC fatal error: wrong stream creation tag.");
      LOG(ERROR) << latest_result_.status();
      fail_flag = true;
    }
  } else if (stream_cq_state == grpc::CompletionQueue::TIMEOUT) {
    latest_result_ = Status(util::error::ABORTED,
                            "gRPC error: Stream creation timed out.");
    LOG(ERROR) << latest_result_.status();
    fail_flag = true;
  } else if (stream_cq_state == grpc::CompletionQueue::SHUTDOWN) {
    latest_result_ = Status(util::error::ABORTED,
                            "gRPC gRPC server shuted down the connection.");
    LOG(ERROR) << latest_result_.status();
    fail_flag = true;
  }

  // Prepares and sends speech recognition configurations.
  gspeech::StreamingRecognizeRequest config_request;
  auto* streaming_config = config_request.mutable_streaming_config();
  streaming_config->set_single_utterance(false);
  streaming_config->set_interim_results(true);
  streaming_config->mutable_config()->set_encoding(RecognitionConfig::LINEAR16);
  streaming_config->mutable_config()->set_sample_rate_hertz(SAMPLE_RATE);
  streaming_config->mutable_config()->set_language_code("en-US");
  streaming_config->mutable_config()->set_max_alternatives(max_alternatives);
  // Prepare the speech context (hints).
  gspeech::SpeechContext* speech_context =
      streaming_config->mutable_config()->add_speech_contexts();
  for (const string& hint : hints) {
    speech_context->add_phrases(hint);
  }

  // Tracks writing.
  uintptr_t last_write_seq = 0;
  uintptr_t write_done_seq = 0;
  bool allow_write = true;
  bool write_done_issued = false;
  bool write_done_finished = false;
  bool final_read_returned = false;

  // Tracking reading.
  uintptr_t last_read_seq = 0;  // Never read before
  gspeech::StreamingRecognizeResponse response;
  bool allow_read = true;

  // If fail_flag is set, all these steps will be skipped and we will start
  // draining the CompletionQueue.
  if (!fail_flag) {
    // Writes the first request, containing the config only, last_write_seq will
    // be updated after calling Write.
    CHECK(allow_write);
    streamer->Write(
        config_request, reinterpret_cast<void*>(last_write_seq = ++cq_seq));
    allow_write = false;
    LOG(INFO) << "Wrote speech config to Google Speech gRPC server with tag "
              << last_write_seq;

    // Starts trying to read from the server, it would only respond when it
    // has something to report back, last_read_seq will be updated after calling
    // Read.
    CHECK(allow_read);
    streamer->Read(&response,
                   reinterpret_cast<void*>(last_read_seq = ++cq_seq));
    allow_read = false;
    LOG(INFO) << "Issued first Read request with tag " << cq_seq;
  }

  // The body of the loop should not take long to finish, so that we can check
  // stop request and deadline.
  // If fail_flag is set, all these steps will be skipped and we will start
  // force draining the CompletionQueue.
  while (!fail_flag && std::chrono::system_clock::now() < deadline) {
    // Try to read everything in the completion_queue
    while (true) {
      // Deadline to wait CompletionQueue to return a result, fix to 10ms. The
      // CompletionQueue should quickly return the results it has, and wait 10ms
      // before it reports it is empty.
      std::chrono::system_clock::time_point cq_deadline
          = std::chrono::system_clock::now() + std::chrono::milliseconds(10);

      // These are the results from CompletionQueue::AsyncNext.
      void* cq_tag = nullptr;
      bool cq_ok = false;

      // Reads from the completion_queue with timeout.
      grpc::CompletionQueue::NextStatus cq_state = completion_queue.AsyncNext(
          &cq_tag, &cq_ok, cq_deadline);

      if (cq_state == grpc::CompletionQueue::GOT_EVENT) {
        if (cq_tag == reinterpret_cast<void*>(last_read_seq)) {
          LOG(INFO) << "Got Read result from completion queue, cq_tag is "
                     << reinterpret_cast<uintptr_t>(cq_tag)
                     << ", cq_ok is " << cq_ok;
          if (!cq_ok) {
            if (write_done_finished) {
              final_read_returned = true;
              break;
            } else {
              fail_flag = true;
              latest_result_ = Status(
                  util::error::INTERNAL, "gRPC Read failed.");
              LOG(ERROR) << latest_result_.status();
              break;
            }
          }

          // Unlock reading if last read is done.
          allow_read = true;

          // We got the read result
          LOG(INFO) << "Got recognition response from server "
                    << response.DebugString();


          if (response.has_error()) {
            latest_result_ = Status(
                util::error::INTERNAL, response.error().message());
            LOG(ERROR) << "Response has error: " << response.error().message();
            fail_flag = true;
            break;
          }

          for (auto& result_record : response.results()) {
            // Put the recognition result to output queue.
            RecognitionResult recog_result;
            recog_result.set_is_final(result_record.is_final());
            recog_result.set_stability(result_record.stability());
            for (auto& alternative : result_record.alternatives()) {
              RecognitionResult::Candidate* can = recog_result.add_candidates();
              can->set_transcript(alternative.transcript());
              can->set_confidence(alternative.confidence());
            }
            latest_result_ = recog_result;
            if (result_queue) {
              result_queue->push(recog_result);
            }
          }

          // Submit another Read request to the the gRPC server.
          CHECK(allow_read);
          streamer->Read(
              &response, reinterpret_cast<void*>(last_read_seq = ++cq_seq));
          allow_read = false;
          LOG(INFO) << "Issued Read request with tag " << last_read_seq;
        } else if (cq_tag == reinterpret_cast<void*>(last_write_seq)) {
          LOG(INFO) << "Got Write result from completion queue, cq_tag is "
                     << reinterpret_cast<uintptr_t>(cq_tag)
                     << ", cq_ok is " << cq_ok;
          if (!cq_ok) {
            fail_flag = true;
            latest_result_ = Status(
                util::error::INTERNAL, "gRPC Write failed.");
            LOG(ERROR) << latest_result_.status();
            break;
          }
          // Unlock writing if last write is done when there are still more
          // samples to write.
          if (!write_done_issued) {
            allow_write = true;
          }
        } else if (cq_tag == reinterpret_cast<void*>(write_done_seq)) {
          LOG(INFO) << "Got WritesDone result from completion queue, cq_tag is "
                     << reinterpret_cast<uintptr_t>(cq_tag)
                     << ", cq_ok is " << cq_ok;
          if (!cq_ok) {
            fail_flag = true;
            latest_result_ = Status(
                util::error::INTERNAL, "gRPC WritesDone failed.");
            LOG(ERROR) << latest_result_.status();
            break;
          }
          write_done_finished = true;
        } else {
          // Got some other results, this is unexpected.
          LOG(ERROR) << "Got unknown result from completion queue, cq_tag is "
                     << reinterpret_cast<uintptr_t>(cq_tag)
                     << ", cq_ok is " << cq_ok;
          if (!cq_ok) {
            fail_flag = true;
            latest_result_ = Status(
                util::error::INTERNAL,
                "Got non-read/write results, and it is a failure.");
            LOG(ERROR) << latest_result_.status();
            break;
          }
        }
      } else if (cq_state == grpc::CompletionQueue::TIMEOUT) {
        LOG(INFO) << "CompletionQueue timeout (expected behavior).";
        break;
      } else if (cq_state == grpc::CompletionQueue::SHUTDOWN) {
        fail_flag = true;
        LOG(ERROR) << "gRPC server shuted down the connection.";
        break;
      }
    }

    if (fail_flag) {
      break;
    }

    if (final_read_returned) {
      break;
    }

    if (allow_write && !write_done_issued) {
      // We can still write.
      if (left_audio_time > 0 && !stop_flag_.load()) {
        // If there is time left, sends audio data to the server.
        gspeech::StreamingRecognizeRequest request;
        // Block some time for reading. We will waste at most this much here.
        // Increase this number should reduce CPU load, but make the program
        // respond to Stop request or new results slower.
        StatusOr<unique_ptr<AudioSample>> pop_result
            = audio_queue->blocking_pop(FLAGS_gspeech_wait_input_timeout_msecs);
        if (pop_result.ok()) {
          unique_ptr<AudioSample> audio_sample = pop_result.ConsumeValueOrDie();
          left_audio_time -= static_cast<double>(audio_sample->size())
                             / (SAMPLE_RATE * 2);
          // And write the chunk to the stream.
          request.set_audio_content(audio_sample->data(), audio_sample->size());
          LOG(INFO) << "Prepare to send " << audio_sample->size()
                    << " bytes of data to the server.";
          CHECK(allow_write);
          streamer->Write(
              request, reinterpret_cast<void*>(last_write_seq = ++cq_seq));
          allow_write = false;
          LOG(INFO) << "Issued Write request with tag " << last_write_seq;
        } else {
          LOG(WARNING) << "Read from audio_queue failed.";
        }
      } else {
        // If this is the last time we to write, issues a done flag.
        CHECK(allow_write);
        CHECK(!write_done_issued);
        streamer->WritesDone(
            reinterpret_cast<void*>(write_done_seq = ++cq_seq));
        LOG(INFO) << "Issued WritesDone request with tag " << write_done_seq;
        allow_write = false;
        write_done_issued = true;
      }
    }
  }

  if (fail_flag) {
    LOG(ERROR) << "gRPC failed, continue to finish the requests. "
               << "There could be more failures, be prepared.";
  }

  bool late_issue_write_done = false;
  if (!write_done_issued) {
    LOG(ERROR) << "WritesDone not issued, but we are not streaming anymore. "
               << "Is max_wait_seconds is too close to max_audio_seconds? "
               << "Is network connection too slow?";
    if (allow_write) {
      streamer->WritesDone(reinterpret_cast<void*>(write_done_seq = ++cq_seq));
      allow_write = false;
      write_done_issued = true;
      late_issue_write_done = true;
      LOG(INFO) << "Issued WritesDone request with tag " << write_done_seq;
    } else {
      LOG(ERROR) << "WritesDone is not yet issued, but not able to write.";
    }
  }

  grpc::Status finish_status;
  uintptr_t finish_seq = 0;
  streamer->Finish(&finish_status,
                   reinterpret_cast<void*>(finish_seq = ++cq_seq));
  LOG(INFO) << "Issued Finish request with tag " << finish_seq;
  completion_queue.Shutdown();

  // Drains the completion_queue.
  bool tried_cancel = false;
  while (true) {
    void* cq_tag = nullptr;
    bool cq_ok = false;
    // Deadline to wait CompletionQueue to return a result, fix to 100ms.
    std::chrono::system_clock::time_point cq_deadline
        = std::chrono::system_clock::now() + std::chrono::milliseconds(100);

    // Read from the completion_queue with some timeout.
    grpc::CompletionQueue::NextStatus cq_state = completion_queue.AsyncNext(
        &cq_tag, &cq_ok, cq_deadline);

    if (cq_state == grpc::CompletionQueue::GOT_EVENT) {
      if (cq_tag == reinterpret_cast<void*>(last_read_seq)) {
        // This should not happen normally because most of the Read requests
        // should have been drained in the previous big while loop. But it is
        // still possible if timeout on max_wait_seconds. In this case, the Read
        // result will be abandoned.
        LOG(WARNING) << "Got unexpected Read from completion queue, cq_tag is "
                     << reinterpret_cast<uintptr_t>(cq_tag)
                     << ", cq_ok is " << cq_ok;
      } else if (cq_tag == reinterpret_cast<void*>(last_write_seq)) {
        LOG(ERROR) << "Got unexpected Write from completion queue, cq_tag is "
                   << reinterpret_cast<uintptr_t>(cq_tag)
                   << ", cq_ok is " << cq_ok;
      } else if (cq_tag == reinterpret_cast<void*>(write_done_seq)) {
        if (late_issue_write_done) {
          LOG(WARNING) << "Got WritesDone (late issued) from completion queue, "
                       << "cq_tag is " << reinterpret_cast<uintptr_t>(cq_tag)
                       << ", cq_ok is " << cq_ok;
        } else {
          LOG(ERROR) << "Got WritesDone from completion queue, cq_tag is "
                     << reinterpret_cast<uintptr_t>(cq_tag)
                     << ", cq_ok is " << cq_ok;
        }
      } else if (cq_tag == reinterpret_cast<void*>(finish_seq)) {
        if (!finish_status.ok()) {
          latest_result_ = Status(
              util::error::INTERNAL, finish_status.error_message());
          LOG(ERROR) << "Finish not OK: " << finish_status.error_message();
        } else {
          LOG(INFO) << "Finish OK.";
        }
      } else {
        LOG(ERROR) << "Got unexpected event from completion queue, cq_tag is "
                   << reinterpret_cast<uintptr_t>(cq_tag)
                   << ", cq_ok is " << cq_ok;
      }
    } else if (cq_state == grpc::CompletionQueue::TIMEOUT) {
      LOG(ERROR) << "CompletionQueue AsyncNext timeout";
      // We still need to wait, otherwise we get gRPC internal error, but we can
      // try force canceling.
      if (!tried_cancel) {
        context.TryCancel();
        tried_cancel = true;
      }
      // TODO(shengye): If this is waiting too long, let's crash the program and
      // let a superviosr restart it.
    } else if (cq_state == grpc::CompletionQueue::SHUTDOWN) {
      // This is expected.
      LOG(INFO) << "CompletionQueue Shutdown";
      break;
    }
  }

  done_flag_.store(true);
  LOG(INFO) << "RecognitionThread finished";
}


}  // namespace speech
}  // namespace cloud
}  // namespace cogrob
