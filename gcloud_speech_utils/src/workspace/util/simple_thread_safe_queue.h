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

#ifndef UTIL_SIMPLE_THREAD_SAFE_QUEUE_H_
#define UTIL_SIMPLE_THREAD_SAFE_QUEUE_H_

#include <condition_variable>
#include <chrono>
#include <mutex>
#include <queue>
#include <utility>

#include "util/statusor.h"

namespace util {

template <typename T> class SimpleThreadSafeQueue {
 private:
  std::queue<T> queue_;
  std::mutex mutex_;
  std::condition_variable condvar_;

 public:
  typedef T value_type;
  template<typename... Args> void push(Args&&... args) {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push(std::forward<Args>(args)...);
    lock.unlock();
    condvar_.notify_one();
  }

  size_t size() {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  T blocking_pop() {
    std::unique_lock<std::mutex> lock(mutex_);
    condvar_.wait(lock, [&] { return !queue_.empty(); });
    T result = std::move(queue_.front());
    queue_.pop();
    return std::move(result);
  }

  util::StatusOr<T> blocking_pop(int wait_ms) {
    std::unique_lock<std::mutex> lock(mutex_);
    std::chrono::milliseconds wait_for_duration(wait_ms);
    condvar_.wait_for(lock, wait_for_duration, [&] { return !queue_.empty(); });
    if (queue_.size() > 0) {
      T result = std::move(queue_.front());
      queue_.pop();
      return std::move(result);
    }
    return util::Status(util::error::UNAVAILABLE, "Size of the queue is 0.");
  }

  util::StatusOr<T> pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.size() > 0) {
      T result = std::move(queue_.front());
      queue_.pop();
      return std::move(result);
    }
    return util::Status(util::error::UNAVAILABLE, "Size of the queue is 0.");
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::queue<T> tmp;
    queue_.swap(tmp);
  }
};

}  // namespace util

#endif  // UTIL_SIMPLE_THREAD_SAFE_QUEUE_H_
